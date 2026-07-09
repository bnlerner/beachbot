#!/usr/bin/env python3
"""Move a MyActuator V3 motor (e.g. X6-60) with absolute or relative position.

Uses official protocol commands over SocketCAN:
  - 0xA4 absolute position
  - 0xA2 speed (for timed trapezoidal velocity profiles)
  - 0x43 acceleration write

Examples:
  # Status only
  python3 tools/move_myactuator.py --status

  # Absolute multi-turn position (degrees)
  python3 tools/move_myactuator.py --position 90

  # Relative move (+20 deg from current)
  python3 tools/move_myactuator.py --relative 20

  # Timed trapezoidal velocity profile lasting 4 seconds
  python3 tools/move_myactuator.py --relative 60 --duration 4
  python3 tools/move_myactuator.py --position 180 --duration 5 --ramp 0.3

  # Slower / smoother single-shot move (no --duration)
  python3 tools/move_myactuator.py --position 0 --max-speed 100 --acceleration 300
"""

from __future__ import annotations

import argparse
import asyncio
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional

# Project root on path (…/src/python)
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivers import can
from drivers.can import enums

# Soft default profile for single-shot position moves (dps²). Protocol: [100, 60000].
_DEFAULT_ACCEL_DPS2 = 500
# How often to stream setpoints during a timed trapezoid (Hz).
_DEFAULT_PROFILE_RATE_HZ = 50.0


@dataclass(frozen=True)
class TrapezoidVelocityProfile:
    """Symmetric trapezoidal velocity profile over a fixed duration.

    Distance is signed (degrees). Velocity is signed dps. Time is seconds.
    Phases: accel [0, t_accel) → cruise [t_accel, t_accel+t_cruise) →
    decel [t_accel+t_cruise, duration] → rest.
    """

    distance_deg: float
    duration_s: float
    t_accel: float
    t_cruise: float
    t_decel: float
    v_peak_dps: float
    accel_dps2: float

    @classmethod
    def plan(
        cls,
        distance_deg: float,
        duration_s: float,
        *,
        ramp_fraction: float = 0.25,
    ) -> "TrapezoidVelocityProfile":
        """Build a profile that covers ``distance_deg`` in exactly ``duration_s``.

        ``ramp_fraction`` is the fraction of total time spent accelerating
        (and the same fraction decelerating). Must be in (0, 0.5).
        """
        if duration_s <= 0:
            raise ValueError("duration must be positive")
        if not (0.0 < ramp_fraction < 0.5):
            raise ValueError("ramp_fraction must be in (0, 0.5)")

        D = float(distance_deg)
        T = float(duration_s)
        if abs(D) < 1e-9:
            return cls(
                distance_deg=0.0,
                duration_s=T,
                t_accel=0.0,
                t_cruise=T,
                t_decel=0.0,
                v_peak_dps=0.0,
                accel_dps2=0.0,
            )

        t_accel = ramp_fraction * T
        t_decel = ramp_fraction * T
        t_cruise = T - t_accel - t_decel
        # D = v * t_cruise + 0.5 * v * t_accel + 0.5 * v * t_decel
        #   = v * (T - 0.5 * (t_accel + t_decel))
        denom = t_cruise + 0.5 * (t_accel + t_decel)
        v_peak = D / denom
        accel = v_peak / t_accel if t_accel > 0 else 0.0

        return cls(
            distance_deg=D,
            duration_s=T,
            t_accel=t_accel,
            t_cruise=t_cruise,
            t_decel=t_decel,
            v_peak_dps=v_peak,
            accel_dps2=accel,
        )

    def velocity(self, t: float) -> float:
        """Signed velocity (dps) at time t seconds from start."""
        if t <= 0.0 or self.v_peak_dps == 0.0:
            return 0.0
        if t < self.t_accel:
            return self.accel_dps2 * t
        if t < self.t_accel + self.t_cruise:
            return self.v_peak_dps
        if t < self.duration_s:
            t_dec = t - self.t_accel - self.t_cruise
            return self.v_peak_dps - self.accel_dps2 * t_dec
        return 0.0

    def position_offset(self, t: float) -> float:
        """Signed displacement (degrees) from the start at time t."""
        if t <= 0.0 or self.v_peak_dps == 0.0:
            return 0.0
        ta, tc, td = self.t_accel, self.t_cruise, self.t_decel
        v, a = self.v_peak_dps, self.accel_dps2
        if t < ta:
            return 0.5 * a * t * t
        s_acc = 0.5 * a * ta * ta
        if t < ta + tc:
            return s_acc + v * (t - ta)
        s_cruise = v * tc
        if t < self.duration_s:
            t_dec = t - ta - tc
            # v*t_dec - 0.5*a*t_dec^2
            return s_acc + s_cruise + v * t_dec - 0.5 * a * t_dec * t_dec
        return self.distance_deg


async def _set_accel_profile(
    can_bus: can.CANSimple,
    node_id: int,
    accel_dps2: int,
    *,
    include_velocity: bool = False,
) -> None:
    """Write accel/decel via official cmd 0x43 (RAM + ROM)."""
    modes = [
        enums.MyActuatorAccelerationType.POSITION_PLANNING_ACCELERATION,
        enums.MyActuatorAccelerationType.POSITION_PLANNING_DECELERATION,
    ]
    if include_velocity:
        modes.extend(
            [
                enums.MyActuatorAccelerationType.VELOCITY_PLANNING_ACCELERATION,
                enums.MyActuatorAccelerationType.VELOCITY_PLANNING_DECELERATION,
            ]
        )
    for mode in modes:
        await can_bus.send(
            can.WriteAccelerationCommand(
                node_id=node_id,
                acceleration_type=mode,
                acceleration_dps2=accel_dps2,
            )
        )
        await asyncio.sleep(0.03)


async def _run_trapezoid_velocity(
    can_bus: can.CANSimple,
    *,
    node_id: int,
    start_deg: float,
    target_deg: float,
    duration_s: float,
    ramp_fraction: float,
    rate_hz: float,
    get_angle,
) -> TrapezoidVelocityProfile:
    """Stream a timed trapezoidal velocity profile.

    1. Stream speed setpoints (cmd 0xA2) along v(t) for ``duration_s``.
    2. Zero speed, then absolute position lock (cmd 0xA4) to the exact target
       and wait until on target (velocity streaming alone drifts a little).
    """
    distance = target_deg - start_deg
    profile = TrapezoidVelocityProfile.plan(
        distance, duration_s, ramp_fraction=ramp_fraction
    )

    motor_accel = max(
        100, min(60000, int(math.ceil(max(abs(profile.accel_dps2) * 2.0, 200))))
    )
    await _set_accel_profile(can_bus, node_id, motor_accel, include_velocity=True)

    print(
        f"Trapezoid: Δ={distance:+.3f}° in {duration_s:.3f}s | "
        f"v_peak={profile.v_peak_dps:+.2f} dps | "
        f"|a|={abs(profile.accel_dps2):.1f} dps² | "
        f"t_acc={profile.t_accel:.3f}s cruise={profile.t_cruise:.3f}s "
        f"dec={profile.t_decel:.3f}s | rate={rate_hz:.0f} Hz"
    )

    dt = 1.0 / rate_hz
    t0 = time.perf_counter()
    next_t = t0
    steps = 0

    while True:
        now = time.perf_counter()
        t = now - t0
        if t >= duration_s:
            break

        # Official API: sendVelocitySetpoint (cmd 0xA2), units dps.
        await can_bus.send(
            can.SpeedControlCommand(
                node_id=node_id, speed=float(profile.velocity(t))
            )
        )
        steps += 1

        if steps % max(1, int(rate_hz)) == 0:
            await can_bus.send(can.ReadMultiTurnAngleMessage(node_id=node_id))
            await can_bus.send(can.ReadMotorStatus2Message(node_id=node_id))

        next_t += dt
        sleep_for = next_t - time.perf_counter()
        if sleep_for > 0:
            await asyncio.sleep(sleep_for)
        else:
            next_t = time.perf_counter()

    # Stop speed loop, then position-lock the exact target.
    await can_bus.send(can.SpeedControlCommand(node_id=node_id, speed=0.0))
    await asyncio.sleep(0.05)
    lock_speed = max(80, int(math.ceil(abs(profile.v_peak_dps) * 2 + 20)))
    print(f"Position lock to {target_deg:.3f}° (max {lock_speed} dps)…")
    await can_bus.send(
        can.PositionControlCommand(
            node_id=node_id,
            position=float(target_deg),
            max_speed=lock_speed,
        )
    )

    # Wait until on target (or timeout) before returning — do not cut short.
    settle_deadline = time.perf_counter() + max(2.0, duration_s * 0.5)
    while time.perf_counter() < settle_deadline:
        await can_bus.send(can.ReadMultiTurnAngleMessage(node_id=node_id))
        await can_bus.send(can.ReadMotorStatus2Message(node_id=node_id))
        await asyncio.sleep(0.1)
        angle = get_angle()
        if angle is not None and abs(angle - target_deg) < 0.5:
            break
    return profile


async def move_motor(
    *,
    node_id: int,
    interface: str,
    position: Optional[float],
    relative: Optional[float],
    max_speed_dps: int,
    acceleration_dps2: int,
    duration_s: Optional[float],
    ramp_fraction: float,
    rate_hz: float,
    status_only: bool,
    settle_s: float,
    hold: bool,
) -> int:
    print(f"Connecting to {interface} (SocketCAN @ 1 Mbit/s)…")
    can_bus = can.CANSimple(
        can_interface=enums.CANInterface(interface),
        bustype=enums.BusType.SOCKET_CAN,
    )

    last_angle: Optional[float] = None
    last_status1: Optional[can.ReadMotorStatus1Message] = None
    last_status2: Optional[can.ReadMotorStatus2Message] = None

    async def on_angle(msg: can.ReadMultiTurnAngleMessage) -> None:
        nonlocal last_angle
        if msg.node_id != node_id:
            return
        last_angle = msg.angle

    async def on_status1(msg: can.ReadMotorStatus1Message) -> None:
        nonlocal last_status1
        if msg.node_id != node_id:
            return
        last_status1 = msg

    async def on_status2(msg: can.ReadMotorStatus2Message) -> None:
        nonlocal last_status2
        if msg.node_id != node_id:
            return
        last_status2 = msg

    can_bus.register_callbacks(
        (can.ReadMultiTurnAngleMessage, on_angle),
        (can.ReadMotorStatus1Message, on_status1),
        (can.ReadMotorStatus2Message, on_status2),
    )
    listen_task = asyncio.create_task(can_bus.listen())

    async def query_status() -> None:
        await can_bus.send(can.ReadMotorStatus1Message(node_id=node_id))
        await can_bus.send(can.ReadMotorStatus2Message(node_id=node_id))
        await can_bus.send(can.ReadMultiTurnAngleMessage(node_id=node_id))
        await asyncio.sleep(0.15)

    def print_status(prefix: str = "") -> None:
        if last_status1 is not None:
            print(
                f"{prefix}status1: temp={last_status1.temperature}°C "
                f"V={last_status1.voltage:.1f} "
                f"err=0x{last_status1.error_state:04x} "
                f"brake_released={last_status1.brake_released}"
            )
        if last_status2 is not None:
            print(
                f"{prefix}status2: temp={last_status2.temperature}°C "
                f"I={last_status2.torque_current:.2f}A "
                f"speed={last_status2.speed} dps "
                f"angle≈{last_status2.angle}°"
            )
        if last_angle is not None:
            print(f"{prefix}multi-turn angle: {last_angle:.3f}°")
        elif last_status1 is None and last_status2 is None:
            print(f"{prefix}(no reply from node_id={node_id})")

    try:
        print(f"Querying motor id={node_id}…")
        await query_status()
        print_status()

        if last_status1 is None and last_angle is None:
            print(
                f"Error: no response from motor id={node_id} on {interface}.\n"
                "  Check power, CAN wiring, bitrate (1 Mbit/s), and:\n"
                "    systemctl status can_setup.service\n"
                "    ip link show can0"
            )
            return 1

        if last_status1 is not None and last_status1.error_state != 0:
            print(
                f"Warning: motor reports error 0x{last_status1.error_state:04x}; "
                "continuing anyway."
            )

        if status_only:
            return 0

        if last_angle is None:
            print("Error: need multi-turn angle feedback before moving.")
            return 1
        start_deg = last_angle

        if relative is not None:
            target = start_deg + relative
            print(f"Relative move {relative:+.3f}° → target {target:.3f}°")
        else:
            assert position is not None
            target = position
            print(f"Absolute move to {target:.3f}° (from {start_deg:.3f}°)")

        # Official API: releaseBrake before motion.
        print("Releasing brake…")
        await can_bus.send(can.SystemBrakeReleaseCommand(node_id=node_id))
        await asyncio.sleep(0.1)

        if duration_s is not None:
            # Timed trapezoidal velocity profile (software-streamed 0xA2).
            await _run_trapezoid_velocity(
                can_bus,
                node_id=node_id,
                start_deg=start_deg,
                target_deg=target,
                duration_s=duration_s,
                ramp_fraction=ramp_fraction,
                rate_hz=rate_hz,
                get_angle=lambda: last_angle,
            )
        else:
            # Single-shot absolute position (cmd 0xA4) with soft accel.
            print(
                f"Setting position accel/decel to {acceleration_dps2} dps² "
                f"(cmd 0x43)…"
            )
            await _set_accel_profile(can_bus, node_id, acceleration_dps2)
            print(
                f"Position command to {target:.3f}° @ max {max_speed_dps} dps "
                f"(cmd 0xA4)"
            )
            await can_bus.send(
                can.PositionControlCommand(
                    node_id=node_id,
                    position=float(target),
                    max_speed=int(max_speed_dps),
                )
            )
            deadline = time.perf_counter() + settle_s
            while time.perf_counter() < deadline:
                await can_bus.send(can.ReadMultiTurnAngleMessage(node_id=node_id))
                await can_bus.send(can.ReadMotorStatus2Message(node_id=node_id))
                await asyncio.sleep(0.1)
                if last_angle is not None and abs(last_angle - target) < 0.5:
                    if last_status2 is not None and abs(last_status2.speed) < 5:
                        break

        print("After move:")
        await query_status()
        print_status("  ")

        if last_angle is not None:
            err = last_angle - target
            print(f"  position error: {err:+.3f}°")

        if not hold:
            print("Sending stop (0x81)…")
            await can_bus.send(can.MotorStopCommand(node_id=node_id))
        else:
            print("Holding (--hold).")

        return 0

    except Exception as e:
        print(f"Error: {e}")
        print("If CAN is down: sudo systemctl restart can_setup.service")
        return 1
    finally:
        listen_task.cancel()
        try:
            await listen_task
        except asyncio.CancelledError:
            pass
        can_bus.shutdown()


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Move a MyActuator V3 motor. "
            "Use --duration for a timed trapezoidal velocity profile."
        )
    )
    p.add_argument(
        "--id",
        type=int,
        default=1,
        help="Motor CAN node ID (1–32). Default: 1",
    )
    p.add_argument(
        "--interface",
        default="can0",
        help="SocketCAN interface name. Default: can0",
    )
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument(
        "--position",
        "-p",
        type=float,
        help="Absolute multi-turn position target in degrees.",
    )
    g.add_argument(
        "--relative",
        "-r",
        type=float,
        help="Relative move in degrees from current multi-turn angle.",
    )
    g.add_argument(
        "--status",
        "-s",
        action="store_true",
        help="Only read and print status; do not move.",
    )
    p.add_argument(
        "--duration",
        "-d",
        type=float,
        default=None,
        help=(
            "If set, run a trapezoidal velocity profile that lasts this many "
            "seconds (covers the requested distance in exactly this time)."
        ),
    )
    p.add_argument(
        "--ramp",
        type=float,
        default=0.25,
        help=(
            "With --duration: fraction of total time for accel (and same for "
            "decel). Must be in (0, 0.5). Default: 0.25 (25%% accel, 50%% "
            "cruise, 25%% decel)."
        ),
    )
    p.add_argument(
        "--rate",
        type=float,
        default=_DEFAULT_PROFILE_RATE_HZ,
        help=(
            f"With --duration: setpoint stream rate in Hz. "
            f"Default: {_DEFAULT_PROFILE_RATE_HZ:.0f}"
        ),
    )
    p.add_argument(
        "--max-speed",
        type=int,
        default=200,
        help=(
            "Without --duration: max speed for single-shot position move "
            "(dps). Default: 200"
        ),
    )
    p.add_argument(
        "--acceleration",
        "--accel",
        type=int,
        default=_DEFAULT_ACCEL_DPS2,
        dest="acceleration",
        help=(
            "Without --duration: position accel/decel in dps² (cmd 0x43). "
            f"Range 100–60000. Default: {_DEFAULT_ACCEL_DPS2}."
        ),
    )
    p.add_argument(
        "--settle",
        type=float,
        default=8.0,
        help="Without --duration: seconds to wait after position cmd. Default: 8",
    )
    p.add_argument(
        "--hold",
        action="store_true",
        help="Do not send stop after the move.",
    )
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    if not (1 <= args.id <= 32):
        print("Error: --id must be between 1 and 32")
        return 1
    if args.max_speed <= 0:
        print("Error: --max-speed must be positive")
        return 1
    if not (100 <= args.acceleration <= 60000):
        print("Error: --acceleration must be between 100 and 60000 dps²")
        return 1
    if args.duration is not None and args.duration <= 0:
        print("Error: --duration must be positive")
        return 1
    if not (0.0 < args.ramp < 0.5):
        print("Error: --ramp must be in (0, 0.5)")
        return 1
    if args.rate <= 0:
        print("Error: --rate must be positive")
        return 1

    try:
        return asyncio.run(
            move_motor(
                node_id=args.id,
                interface=args.interface,
                position=args.position,
                relative=args.relative,
                max_speed_dps=args.max_speed,
                acceleration_dps2=args.acceleration,
                duration_s=args.duration,
                ramp_fraction=args.ramp,
                rate_hz=args.rate,
                status_only=args.status,
                settle_s=args.settle,
                hold=args.hold,
            )
        )
    except KeyboardInterrupt:
        print("\nCancelled.")
        return 130


if __name__ == "__main__":
    sys.exit(main())
