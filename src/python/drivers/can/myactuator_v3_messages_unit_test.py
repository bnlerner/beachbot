"""Unit tests for MyActuator V3 protocol encode/decode (offline, no hardware)."""

from __future__ import annotations

import can
import pytest

from drivers.can import enums, myactuator_v3_messages as m


def _rx(cmd: int, data: list[int], node_id: int = 1) -> can.Message:
    payload = bytes([cmd] + data)
    assert len(payload) == 8
    return can.Message(
        arbitration_id=0x240 + node_id,
        data=payload,
        is_extended_id=False,
    )


class TestExistingApis:
    def test_position_control_encode(self) -> None:
        msg = m.PositionControlCommand(node_id=1, position=360.0, max_speed=500)
        frame = msg.as_can_message()
        assert frame.arbitration_id == 0x141
        assert list(frame.data) == [0xA4, 0x00, 0xF4, 0x01, 0xA0, 0x8C, 0x00, 0x00]

    def test_position_control_negative_encode(self) -> None:
        msg = m.PositionControlCommand(node_id=1, position=-360.0, max_speed=500)
        assert list(msg.as_can_message().data) == [
            0xA4,
            0x00,
            0xF4,
            0x01,
            0x60,
            0x73,
            0xFF,
            0xFF,
        ]

    def test_speed_control_encode(self) -> None:
        msg = m.SpeedControlCommand(node_id=1, speed=100.0)
        assert list(msg.as_can_message().data) == [
            0xA2,
            0x00,
            0x00,
            0x00,
            0x10,
            0x27,
            0x00,
            0x00,
        ]

    def test_torque_control_encode(self) -> None:
        msg = m.TorqueControlCommand(node_id=1, torque_current=1.0)
        assert list(msg.as_can_message().data) == [
            0xA1,
            0x00,
            0x00,
            0x00,
            0x64,
            0x00,
            0x00,
            0x00,
        ]

    def test_write_acceleration_encode(self) -> None:
        msg = m.WriteAccelerationCommand(
            node_id=1,
            acceleration_type=enums.MyActuatorAccelerationType.POSITION_PLANNING_ACCELERATION,
            acceleration_dps2=10000,
        )
        assert list(msg.as_can_message().data) == [
            0x43,
            0x00,
            0x00,
            0x00,
            0x10,
            0x27,
            0x00,
            0x00,
        ]

    def test_status1_parse(self) -> None:
        raw = _rx(0x9A, [0x32, 0x00, 0x01, 0xE5, 0x01, 0x04, 0x00])
        parsed = m.MyactuatorReadMotorStatus1Message.from_can_message(raw)
        assert parsed.node_id == 1
        assert parsed.temperature == 50
        assert parsed.brake_released is True
        assert parsed.voltage == pytest.approx(48.5)
        assert parsed.error_state == 0x0004

    def test_status2_parse(self) -> None:
        raw = _rx(0x9C, [0x32, 0x64, 0x00, 0xF4, 0x01, 0x2D, 0x00])
        parsed = m.ReadMotorStatus2Message.from_can_message(raw)
        assert parsed.temperature == 50
        assert parsed.torque_current == pytest.approx(1.0)
        assert parsed.speed == 500
        assert parsed.angle == 45

    def test_multi_turn_angle_parse(self) -> None:
        raw = _rx(0x92, [0x00, 0x00, 0x00, 0xA0, 0x8C, 0x00, 0x00])
        parsed = m.ReadMultiTurnAngleMessage.from_can_message(raw)
        assert parsed.angle == pytest.approx(360.0)

    def test_brake_release_encode(self) -> None:
        msg = m.SystemBrakeReleaseCommand(node_id=1)
        assert list(msg.as_can_message().data) == [0x77, 0, 0, 0, 0, 0, 0, 0]


class TestNewProtocolMessages:
    def test_pid_read_parse(self) -> None:
        raw = _rx(0x30, [0x00, 0x55, 0x19, 0x55, 0x19, 0x55, 0x19])
        parsed = m.ReadPIDParametersMessage.from_can_message(raw)
        assert parsed.current_kp == 85
        assert parsed.current_ki == 25
        assert parsed.speed_kp == 85
        assert parsed.position_ki == 25

    def test_pid_write_ram_encode(self) -> None:
        msg = m.WritePIDParametersToRAMMessage(
            node_id=1,
            current_kp=85,
            current_ki=25,
            speed_kp=85,
            speed_ki=25,
            position_kp=85,
            position_ki=25,
        )
        assert list(msg.as_can_message().data) == [
            0x31,
            0x00,
            0x55,
            0x19,
            0x55,
            0x19,
            0x55,
            0x19,
        ]

    def test_status3_parse(self) -> None:
        raw = _rx(0x9D, [0x32, 0xC2, 0x0B, 0x10, 0xFA, 0xC0, 0xF9])
        parsed = m.ReadMotorStatus3Message.from_can_message(raw)
        assert parsed.temperature == 50
        assert parsed.current_phase_a == pytest.approx(30.1, abs=0.05)
        assert parsed.current_phase_b == pytest.approx(-15.2, abs=0.05)
        assert parsed.current_phase_c == pytest.approx(-16.0, abs=0.05)

    def test_single_turn_encoder_parse(self) -> None:
        raw = _rx(0x90, [0x00, 0x33, 0x08, 0xBE, 0x2C, 0x8B, 0x24])
        parsed = m.ReadSingleTurnEncoderMessage.from_can_message(raw)
        assert parsed.position == 2099
        assert parsed.raw_position == 11454
        assert parsed.offset == 9355

    def test_single_turn_angle_parse(self) -> None:
        raw = _rx(0x94, [0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27])
        parsed = m.ReadSingleTurnAngleMessage.from_can_message(raw)
        assert parsed.angle == pytest.approx(100.0)

    def test_encoder_position_parse(self) -> None:
        raw = _rx(0x60, [0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00])
        parsed = m.ReadMultiTurnEncoderPositionMessage.from_can_message(raw)
        assert parsed.position == 10000

    def test_motor_power_parse(self) -> None:
        raw = _rx(0x71, [0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x07])
        parsed = m.MotorPowerAcquisitionCommand.from_can_message(raw)
        assert parsed.power == pytest.approx(200.0)

    def test_runtime_parse(self) -> None:
        raw = _rx(0xB1, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10])
        parsed = m.SystemRuntimeReadCommand.from_can_message(raw)
        assert parsed.runtime_ms == 0x10000000

    def test_version_parse(self) -> None:
        raw = _rx(0xB2, [0x00, 0x00, 0x00, 0x2E, 0x89, 0x34, 0x01])
        parsed = m.VersionAcquisitionCommand.from_can_message(raw)
        assert parsed.version_date == 20220206
        assert parsed.version_datetime().year == 2022

    def test_motor_model_parse(self) -> None:
        raw = _rx(0xB5, [0x58, 0x38, 0x53, 0x32, 0x56, 0x31, 0x30])
        parsed = m.MotorModelReadingCommand.from_can_message(raw)
        assert parsed.model == "X8S2V10"

    def test_timeout_encode(self) -> None:
        msg = m.CommunicationInterruptionProtectionCommand(
            node_id=1, timeout_ms=1000
        )
        assert list(msg.as_can_message().data) == [
            0xB3,
            0x00,
            0x00,
            0x00,
            0xE8,
            0x03,
            0x00,
            0x00,
        ]

    def test_baud_encode(self) -> None:
        msg = m.CommunicationBaudRateSettingCommand(
            node_id=1, baud_rate=enums.MyActuatorCanBaudRate.MBPS_1
        )
        assert list(msg.as_can_message().data) == [
            0xB4,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x01,
        ]

    def test_single_turn_position_encode(self) -> None:
        msg = m.SingleTurnPositionControlCommand(
            node_id=1, spin_direction=0, max_speed=500, position=90.0
        )
        data = list(msg.as_can_message().data)
        assert data[0] == 0xA6
        assert data[1] == 0
        assert data[2:4] == [0xF4, 0x01]  # 500
        assert data[4:6] == [0x28, 0x23]  # 9000 = 90.00°
        assert data[6:8] == [0, 0]

    def test_position_reply_feedback(self) -> None:
        raw = _rx(0xA4, [0x32, 0x64, 0x00, 0xF4, 0x01, 0x2D, 0x00])
        parsed = m.PositionControlCommand.from_can_message(raw)
        assert parsed.temperature == 50
        assert parsed.torque_current == pytest.approx(1.0)
        assert parsed.speed == 500
        assert parsed.angle == 45

    def test_operating_mode_parse(self) -> None:
        raw = _rx(0x70, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03])
        parsed = m.SystemOperatingModeAcquisitionCommand.from_can_message(raw)
        assert parsed.operating_mode == enums.MyActuatorV3OperatingMode.POSITION_LOOP_CONTROL

    def test_canid_arbitration(self) -> None:
        msg = m.CANIDCommand(node_id=1, read_write_flag="write", can_id=2)
        frame = msg.as_can_message()
        assert frame.arbitration_id == 0x300
        assert list(frame.data) == [0x79, 0, 0, 0, 0, 0, 0, 2]

    def test_factory(self) -> None:
        msg = m.create_myactuator_message(0xA4, node_id=3, position=10.0, max_speed=100)
        assert isinstance(msg, m.PositionControlCommand)
        assert msg.node_id == 3
        with pytest.raises(ValueError):
            m.create_myactuator_message(0xFF)

    def test_package_exports(self) -> None:
        from drivers import can as can_pkg

        assert can_pkg.PositionControlCommand is m.PositionControlCommand
        assert can_pkg.ReadMotorStatus3Message is m.ReadMotorStatus3Message
        assert can_pkg.ReadPIDParametersMessage is m.ReadPIDParametersMessage
