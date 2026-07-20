#!/usr/bin/env bash
set -euo pipefail

ROBOT_ETH_IP="${1:-}"
SSID="${2:-}"
ROBOT_WIFI_IF="${3:-}"
ROBOT_USER="${GO2_SSH_USER:-unitree}"

if [[ -z "${ROBOT_ETH_IP}" || -z "${SSID}" ]]; then
  cat >&2 <<EOF
Usage:
  $0 ROBOT_ETHERNET_IP SSID [ROBOT_WIFI_INTERFACE]

Environment:
  GO2_SSH_USER=${ROBOT_USER}

The robot must remain connected over Ethernet during this operation.
SSH, sudo, and Wi-Fi credentials are requested interactively.
EOF
  exit 2
fi

if [[ ! "${ROBOT_ETH_IP}" =~ ^[0-9a-fA-F:.]+$ ]]; then
  echo "Invalid robot IP: ${ROBOT_ETH_IP}" >&2
  exit 2
fi
if [[ "${SSID}" == *$'\n'* || "${SSID}" == *$'\r'* ]]; then
  echo "SSID must not contain newlines." >&2
  exit 2
fi
if [[ -n "${ROBOT_WIFI_IF}" && ! "${ROBOT_WIFI_IF}" =~ ^[a-zA-Z0-9_.:-]+$ ]]; then
  echo "Invalid robot Wi-Fi interface: ${ROBOT_WIFI_IF}" >&2
  exit 2
fi

if ip -o -4 addr show 2>/dev/null | awk '{split($4, a, "/"); print a[1]}' | grep -Fxq "${ROBOT_ETH_IP}"; then
  echo "Refusing to SSH to ${ROBOT_ETH_IP}: this address belongs to the laptop." >&2
  echo "Use the Go2 computer's Ethernet IP, not the address configured on the laptop NIC." >&2
  exit 2
fi

printf '[ROBOT WIFI] Ethernet SSH target: %s@%s\n' "${ROBOT_USER}" "${ROBOT_ETH_IP}"
printf '[ROBOT WIFI] Target SSID: %s\n' "${SSID}"
printf '[ROBOT WIFI] Keep Ethernet connected until the Wi-Fi peer test passes.\n'

REMOTE_SCRIPT_PATH="/tmp/go2_wifi_setup_$$.sh"
SSH_CONTROL_PATH="/tmp/go2_wifi_ssh_${USER:-user}_$$"

cleanup() {
  ssh -S "${SSH_CONTROL_PATH}" -O exit "${ROBOT_USER}@${ROBOT_ETH_IP}" >/dev/null 2>&1 || true
  rm -f "${SSH_CONTROL_PATH}"
}
trap cleanup EXIT

echo "[ROBOT WIFI] Opening one SSH session. Enter the Go2 computer password."
ssh -M -S "${SSH_CONTROL_PATH}" -o ControlPersist=60 -N -f \
  "${ROBOT_USER}@${ROBOT_ETH_IP}"

ssh -S "${SSH_CONTROL_PATH}" "${ROBOT_USER}@${ROBOT_ETH_IP}" \
  "cat > '${REMOTE_SCRIPT_PATH}' && chmod 700 '${REMOTE_SCRIPT_PATH}'" <<'REMOTE_SCRIPT'
set -euo pipefail

ssid="$1"
requested_if="$2"

if ! command -v nmcli >/dev/null 2>&1; then
  echo "The Go2 computer does not provide nmcli. Configure its USB Wi-Fi adapter manually." >&2
  exit 2
fi

wifi_if="${requested_if}"
if [[ -z "${wifi_if}" ]]; then
  wifi_if="$(
    nmcli -t -f DEVICE,TYPE device status \
      | awk -F: '$2 == "wifi" {print $1; exit}'
  )"
fi
if [[ -z "${wifi_if}" ]]; then
  echo "No Wi-Fi interface was detected on the Go2 computer." >&2
  echo "Check that the USB dongle is supported by the robot kernel and appears in 'ip -br link'." >&2
  exit 2
fi

echo "[ROBOT WIFI] Interface: ${wifi_if}"
sudo nmcli radio wifi on
sudo nmcli device set "${wifi_if}" managed yes
sudo nmcli device wifi rescan ifname "${wifi_if}" || true
echo "[ROBOT WIFI] Connecting to '${ssid}'. NetworkManager may prompt for the WPA password."
sudo nmcli --ask device wifi connect "${ssid}" ifname "${wifi_if}"

echo
echo "[ROBOT WIFI] Connected state:"
nmcli -t -f GENERAL.DEVICE,GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS device show "${wifi_if}"
echo
echo "[ROBOT WIFI] Record the IP4.ADDRESS value. Use it as ROBOT_WIFI_IP in the laptop peer test."
REMOTE_SCRIPT

printf -v quoted_ssid '%q' "${SSID}"
printf -v quoted_wifi_if '%q' "${ROBOT_WIFI_IF}"
ssh -S "${SSH_CONTROL_PATH}" -t "${ROBOT_USER}@${ROBOT_ETH_IP}" \
  "bash '${REMOTE_SCRIPT_PATH}' ${quoted_ssid} ${quoted_wifi_if}; status=\$?; rm -f '${REMOTE_SCRIPT_PATH}'; exit \$status"
