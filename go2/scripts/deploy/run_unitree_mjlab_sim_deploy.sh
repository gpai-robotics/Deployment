#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT=$REPO
echo "REPO      = $REPO"
echo "REPO_ROOT = $REPO_ROOT"
SIM_BIN="${REPO_ROOT}/go2/reference_repos/Unitree_mjlab_repo/unitree_rl_mjlab/simulate/build/unitree_mujoco"
CTRL_BIN="${REPO_ROOT}/go2/reference_repos/Unitree_mjlab_repo/unitree_rl_mjlab/deploy/robots/go2/build/go2_ctrl"
GO2_SCENE="${REPO_ROOT}/go2/reference_repos/Unitree_mjlab_repo/unitree_rl_mjlab/src/assets/robots/unitree_go2/xmls/scene_go2.xml"
MUJOCO_PYTHON="${MUJOCO_PYTHON:-python}"
TERRAIN_DIR="${REPO_ROOT}/go2/artifacts/mujoco_mjlab_terrains"
source "${REPO_ROOT}/go2/scripts/deploy/go2_network.sh"

usage() {
  cat <<EOF
Usage:
  $0 sim [scene.xml]
  $0 sim-terrain TERRAIN [DIFFICULTY] [SEED]
  $0 sim-gamepad [scene.xml]
  $0 controller
  $0 controller-fixed
  $0 validate
  $0 activate
  $0 network-status [ethernet|wifi|auto|INTERFACE] [INTERFACE]
  $0 wifi-scan [INTERFACE]
  $0 wifi-connect SSID [INTERFACE]
  $0 robot-wifi-connect ROBOT_ETHERNET_IP SSID [ROBOT_WIFI_INTERFACE]
  $0 wifi-peer ROBOT_WIFI_IP [INTERFACE]
  $0 dds-probe [ethernet|wifi|auto|INTERFACE] [INTERFACE]
  $0 monitor [ethernet|wifi|auto|INTERFACE] [LABEL] [INTERFACE]
  $0 hardware [ethernet|wifi|auto|INTERFACE] [INTERFACE]

Run the controller and simulator in separate terminals. Start the controller first.

The 'controller' mode injects the simulated command. Configure it with:
  MJLAB_SCRIPT_VX=0.3
  MJLAB_SCRIPT_VY=0.0
  MJLAB_SCRIPT_YAW=0.0

Use 'sim-gamepad' only when /dev/input/js0 is available.

The 'hardware' mode runs the real C++ FSM controller without simulation
autostart injection. Use the wireless remote for:
  L2 + up  -> FixStand
  R2 + A   -> Velocity
  L2 + B   -> Passive

The active runtime is intentionally restricted to:
  go2_blind_rough_asymppo_mjlab_v1_candidate

The 'monitor' mode is a read-only LowState/LowCmd subscriber. Run it in a
separate terminal while the hardware controller is active.

Network defaults:
  Ethernet: GO2_ETH_IF=${GO2_ETH_IF_DEFAULT}
  Wi-Fi:    GO2_WIFI_IF=${GO2_WIFI_IF_DEFAULT:-<auto-detect>}

Examples:
  $0 hardware ethernet
  $0 hardware wifi
  $0 hardware wifi wlx8c86dd5c83c1
  $0 dds-probe wifi
  $0 monitor wifi asymppo_wifi
  $0 network-status auto
EOF
}

case "${1:-}" in
  sim)
    scene="${2:-${GO2_SCENE}}"
    exec "${SIM_BIN}" \
      --network=lo \
      --robot=go2 \
      --scene="${scene}" \
      --use_joystick=0
    ;;
  sim-terrain)
    terrain="${2:-random_rough}"
    difficulty="${3:-0.5}"
    seed="${4:-42}"
    stem="go2_mjlab_unitree_runtime_${terrain}_seed${seed}_r1_c1_d${difficulty}-${difficulty}"
    scene="${TERRAIN_DIR}/${stem}.xml"
    if [[ ! -f "${scene}" ]]; then
      "${MUJOCO_PYTHON}" \
        "${REPO_ROOT}/go2/scripts/deploy/materialize_mjlab_mujoco_terrain.py" \
        --robot-model unitree_mjlab \
        --terrain "${terrain}" \
        --seed "${seed}" \
        --rows 1 \
        --cols 1 \
        --difficulty-min "${difficulty}" \
        --difficulty-max "${difficulty}" \
        --spawn-row 0 \
        --spawn-col 0 \
        --name "${stem}"
    fi
    if [[ ! -f "${scene}" ]]; then
      echo "Failed to create portable terrain XML: ${scene}" >&2
      exit 1
    fi
    printf '%s\n' \
      "NOTE: This is backend-specific exploratory OOD testing." \
      "It is not the canonical cross-simulator deployment gate:" \
      "- runtime physics dt is 0.002 s, not matched-profile 0.005 s" \
      "- matched observation/action/command delays are not enabled" \
      "- canonical Isaac DCMotor torque-speed emulation is not enabled"
    exec "${SIM_BIN}" \
      --network=lo \
      --robot=go2 \
      --scene="${scene}" \
      --use_joystick=0
    ;;
  sim-gamepad)
    scene="${2:-${GO2_SCENE}}"
    exec "${SIM_BIN}" \
      --network=lo \
      --robot=go2 \
      --scene="${scene}" \
      --use_joystick=1
    ;;
  controller|controller-fixed)
    export UNITREE_MJLAB_AUTOSTART=1
    export UNITREE_MJLAB_VX="${MJLAB_SCRIPT_VX:-0.3}"
    export UNITREE_MJLAB_VY="${MJLAB_SCRIPT_VY:-0.0}"
    export UNITREE_MJLAB_YAW="${MJLAB_SCRIPT_YAW:-0.0}"
    export UNITREE_MJLAB_BUTTON_DURATION="${MJLAB_BUTTON_DURATION:-1.0}"
    export UNITREE_MJLAB_REPEAT_PERIOD="${MJLAB_TRANSITION_REPEAT:-12.0}"
    export UNITREE_MJLAB_FIXSTAND_TIME="${MJLAB_FIXSTAND_TIME:-0.1}"
    export UNITREE_MJLAB_VELOCITY_TIME="${MJLAB_VELOCITY_TIME:-4.0}"
    if [[ "${1}" == "controller" ]]; then
      export UNITREE_MJLAB_TELEOP=1
      export UNITREE_MJLAB_TELEOP_LINEAR="${MJLAB_TELEOP_LINEAR:-0.8}"
      export UNITREE_MJLAB_TELEOP_LATERAL="${MJLAB_TELEOP_LATERAL:-0.6}"
      export UNITREE_MJLAB_TELEOP_YAW="${MJLAB_TELEOP_YAW:-0.8}"
    else
      export UNITREE_MJLAB_TELEOP=0
    fi
    exec "${CTRL_BIN}" --network=lo
    ;;
  validate)
    exec "${MUJOCO_PYTHON}" \
      "${REPO_ROOT}/go2/scripts/deploy/validate_unitree_mjlab_go2_fsm_runtime.py" \
      --strict-fixstand-gains \
      --json-out "${REPO_ROOT}/go2/artifacts/deployment_validation/active_unitree_mjlab_fsm_runtime_audit.json"
    ;;
  activate)
    if [[ -n "${2:-}" && "${2}" != "asym" ]]; then
      echo "Only the AsymPPO candidate is available from the active launcher." >&2
      echo "Legacy candidates remain archived but are intentionally not switchable here." >&2
      exit 2
    fi
    bundle="${REPO_ROOT}/go2/policies/go2_blind_rough_asymppo_mjlab_v1_candidate"
    runtime_name="go2_blind_rough_asymppo_mjlab_v1_candidate"
    exec "${MUJOCO_PYTHON}" \
      "${REPO_ROOT}/go2/scripts/deploy/prepare_unitree_rl_mjlab_go2_runtime.py" \
      --bundle-dir "${bundle}" \
      --runtime-name "${runtime_name}" \
      --activate \
      --strict-fixstand-gains \
      --force
    ;;
  network-status)
    selector="${2:-auto}"
    explicit_interface="${3:-}"
    network="$(go2_resolve_network_interface "${selector}" "${explicit_interface}")"
    go2_print_network_status "${network}"
    ;;
  wifi-scan)
    explicit_interface="${2:-}"
    if ! command -v nmcli >/dev/null 2>&1; then
      echo "NetworkManager CLI (nmcli) is required for Wi-Fi scanning." >&2
      exit 2
    fi
    network="$(go2_resolve_network_interface wifi "${explicit_interface}")"
    nmcli radio wifi on
    nmcli device set "${network}" managed yes
    exec nmcli --colors yes --fields IN-USE,SSID,SIGNAL,SECURITY device wifi list \
      ifname "${network}" --rescan yes
    ;;
  wifi-connect)
    ssid="${2:-}"
    explicit_interface="${3:-}"
    if [[ -z "${ssid}" ]]; then
      echo "Missing SSID. Example: $0 wifi-connect Go2Network" >&2
      exit 2
    fi
    if ! command -v nmcli >/dev/null 2>&1; then
      echo "NetworkManager CLI (nmcli) is required for interactive Wi-Fi setup." >&2
      exit 2
    fi
    network="$(go2_resolve_network_interface wifi "${explicit_interface}")"
    echo "[NETWORK] Connecting ${network} to '${ssid}'."
    echo "[NETWORK] NetworkManager will prompt for credentials if required."
    nmcli radio wifi on
    nmcli device set "${network}" managed yes
    exec nmcli --ask device wifi connect "${ssid}" ifname "${network}"
    ;;
  robot-wifi-connect)
    robot_eth_ip="${2:-}"
    ssid="${3:-}"
    robot_wifi_if="${4:-}"
    if [[ -z "${robot_eth_ip}" || -z "${ssid}" ]]; then
      echo "Usage: $0 robot-wifi-connect ROBOT_ETHERNET_IP SSID [ROBOT_WIFI_INTERFACE]" >&2
      exit 2
    fi
    exec "${REPO_ROOT}/go2/scripts/deploy/configure_go2_wifi_over_ssh.sh" \
      "${robot_eth_ip}" "${ssid}" "${robot_wifi_if}"
    ;;
  wifi-peer)
    robot_wifi_ip="${2:-}"
    explicit_interface="${3:-}"
    if [[ -z "${robot_wifi_ip}" ]]; then
      echo "Missing robot Wi-Fi IP. Obtain it from robot-wifi-connect or the hotspot client list." >&2
      exit 2
    fi
    if [[ ! "${robot_wifi_ip}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
      echo "Invalid robot Wi-Fi IPv4 address: ${robot_wifi_ip}" >&2
      exit 2
    fi
    network="$(go2_resolve_network_interface wifi "${explicit_interface}")"
    go2_validate_network_interface "${network}"
    if ! command -v ping >/dev/null 2>&1; then
      echo "Missing required command: ping" >&2
      exit 2
    fi
    echo "[PEER] Checking route to ${robot_wifi_ip} through ${network}."
    route="$(ip -4 route get "${robot_wifi_ip}" oif "${network}" 2>/dev/null || true)"
    if [[ -z "${route}" ]]; then
      echo "[FAIL] No route to ${robot_wifi_ip} through ${network}." >&2
      exit 2
    fi
    echo "[PEER] ${route}"
    if ! ping -I "${network}" -c 3 -W 1 "${robot_wifi_ip}"; then
      echo "[FAIL] The laptop cannot reach the Go2 Wi-Fi IP." >&2
      echo "[DIAG] Either the robot did not join this WLAN or the hotspot isolates clients." >&2
      exit 2
    fi
    echo "[PASS] Go2 Wi-Fi peer is reachable."
    echo "[NEXT] Run: $0 dds-probe wifi"
    ;;
  dds-probe)
    selector="${2:-ethernet}"
    explicit_interface="${3:-}"
    network="$(go2_resolve_network_interface "${selector}" "${explicit_interface}")"
    go2_validate_network_interface "${network}"
    go2_hw_python="${GO2_HW_PYTHON:-python3}"
    if [[ ! -x "${go2_hw_python}" ]]; then
      echo "Missing hardware Python: ${go2_hw_python}" >&2
      exit 2
    fi
    echo "[SAFETY] Read-only DDS probe. No LowCmd publisher or mode switch."
    if "${go2_hw_python}" "${REPO_ROOT}/go2/scripts/deploy/probe_go2_readonly.py" \
      --net-if "${network}" \
      --duration 3 \
      --print-every 1 \
      --no-sport; then
      echo "[PASS] Go2 LowState DDS packets received on ${network}."
    else
      status=$?
      echo >&2
      echo "[FAIL] No Go2 LowState DDS packets were received on ${network}." >&2
      if go2_is_wireless_interface "${network}"; then
        ssid="$(go2_active_wifi_ssid "${network}" || true)"
        echo "[DIAG] Connected SSID: ${ssid:-<unknown>}" >&2
        echo "[DIAG] Laptop IPv4: $(go2_interface_ipv4 "${network}")" >&2
        echo "[DIAG] The Go2 dongle and this adapter must join the same WLAN." >&2
        echo "[DIAG] The WLAN must permit multicast and client-to-client traffic." >&2
        if [[ "${ssid,,}" == *guest* ]]; then
          echo "[DIAG] '${ssid}' is a guest WLAN and is the likely blocker." >&2
          echo "[DIAG] Use a dedicated hotspot/router or a lab IoT WLAN with AP isolation disabled." >&2
        fi
      fi
      exit "${status}"
    fi
    ;;
  monitor)
    selector="${2:-ethernet}"
    label="${3:-asymppo}"
    explicit_interface="${4:-}"
    network="$(go2_resolve_network_interface "${selector}" "${explicit_interface}")"
    go2_validate_network_interface "${network}"
    exec "${REPO_ROOT}/go2/scripts/deploy/run_go2_realtime_monitor.sh" \
      "${network}" 20 25 "${label}"
    ;;
  hardware)
    selector="${2:-ethernet}"
    explicit_interface="${3:-}"
    network="$(go2_resolve_network_interface "${selector}" "${explicit_interface}")"
    go2_validate_network_interface "${network}"
    unset UNITREE_MJLAB_AUTOSTART
    unset UNITREE_MJLAB_TELEOP
    exec "${CTRL_BIN}" --network="${network}"
    ;;
  *)
    usage
    exit 2
    ;;
esac
