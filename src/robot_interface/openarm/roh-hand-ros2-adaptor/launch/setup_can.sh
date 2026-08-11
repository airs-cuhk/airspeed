#!/usr/bin/env bash
# Canonical CAN setup for the ROH hand + OpenArm stack (run with sudo).
#
# Why this exists: the OHand SDK can only open can1..can16 (port 0 invalid),
# and USB enumeration order is not stable — after a replug the ROH dongles
# can land on can0/can1, breaking both the arms (expect can0/can1) and the
# hands (SDK cannot open can0). This script pins interfaces by USB ID:
#
#   PEAK PCAN-USB Pro FD (0c72:0011, 2ch)  -> can0/can1  (arms)
#   PEAK PCAN-USB        (0c72:000c, 1ch×2) -> can2/can3  (ROH hands)
#
# Caveat: the two ROH dongles are identical — which one is "left" (can2) vs
# "right" (can3) follows enumeration order. Verify per-hand before a session.
set -euo pipefail

if [ "$EUID" -ne 0 ]; then
    echo "run with sudo" >&2
    exit 1
fi

model_id_of() { udevadm info -q property -p "/sys/class/net/$1" 2>/dev/null | sed -n 's/ID_MODEL_ID=//p'; }

# Map current interfaces by USB model id.
ARM_IFS=()
ROH_IFS=()
for path in /sys/class/net/can*; do
    dev="$(basename "$path")"
    case "$(model_id_of "$dev")" in
        0011) ARM_IFS+=("$dev") ;;
        000c) ROH_IFS+=("$dev") ;;
    esac
done

if [ "${#ARM_IFS[@]}" -ne 2 ] || [ "${#ROH_IFS[@]}" -ne 2 ]; then
    echo "expected 2 arm channels (0c72:0011) + 2 ROH channels (0c72:000c);" >&2
    echo "found arms=[${ARM_IFS[*]:-}] roh=[${ROH_IFS[*]:-}]" >&2
    exit 1
fi

# Sort for deterministic assignment.
IFS=$'\n' ARM_IFS=($(sort <<<"${ARM_IFS[*]}")); unset IFS
IFS=$'\n' ROH_IFS=($(sort <<<"${ROH_IFS[*]}")); unset IFS

echo "arms: ${ARM_IFS[0]} ${ARM_IFS[1]} -> can0 can1"
echo "hands: ${ROH_IFS[0]} ${ROH_IFS[1]} -> can2 can3"

# Everything down, then rename through temp names to avoid collisions.
for dev in can0 can1 can2 can3 "${ARM_IFS[@]}" "${ROH_IFS[@]}"; do
    ip link set "$dev" down 2>/dev/null || true
done
ip link set "${ARM_IFS[0]}" name can10
ip link set "${ARM_IFS[1]}" name can11
ip link set "${ROH_IFS[0]}" name can12
ip link set "${ROH_IFS[1]}" name can13
ip link set can10 name can0
ip link set can11 name can1
ip link set can12 name can2
ip link set can13 name can3

# Hands: plain CAN @ 1 Mbps, auto-recover from bus-off.
for dev in can2 can3; do
    ip link set "$dev" type can bitrate 1000000 restart-ms 100
    ip link set "$dev" up
done

# Arms: CAN FD (nominal 1 Mbps, data 5 Mbps) — the Damiao buses use FD
# frames; a plain-CAN link receives nothing (0 RX packets) and the arm
# controller reports the joints as not connected.
for dev in can0 can1; do
    ip link set "$dev" type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
    ip link set "$dev" up
done

ip -details link show can0 | head -3
ip -details link show can2 | head -3
echo "CAN setup complete: arms can0/can1, hands can2/can3"
