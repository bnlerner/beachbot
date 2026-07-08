#!/bin/bash
# Bring up a SocketCAN interface for beachbot (gs_usb / CANable-class adapters).
# Usage: setup_can.sh [iface] [bitrate]
#   iface defaults to can0; bitrate defaults to 1000000.
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-1000000}"
WAIT_SECS="${CAN_WAIT_SECS:-15}"

# Wait for the netdev (USB adapters may appear after boot).
deadline=$((SECONDS + WAIT_SECS))
while [[ ! -d "/sys/class/net/${IFACE}" ]]; do
  if (( SECONDS >= deadline )); then
    echo "setup_can: interface ${IFACE} not present after ${WAIT_SECS}s" >&2
    exit 1
  fi
  sleep 0.2
done

# Idempotent: down (ignore errors), then up at the requested bitrate.
ip link set "${IFACE}" down 2>/dev/null || true
ip link set "${IFACE}" up type can bitrate "${BITRATE}"
ip -details link show "${IFACE}"
echo "setup_can: ${IFACE} up at ${BITRATE} bit/s"
