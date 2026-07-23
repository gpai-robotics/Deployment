#!/usr/bin/env bash

# Shared network selection and validation for the active Go2 DDS tools.

GO2_ETH_IF_DEFAULT="${GO2_ETH_IF:-enp0s31f6}"
GO2_WIFI_IF_DEFAULT="${GO2_WIFI_IF:-}"

go2_is_wireless_interface() {
  local interface="$1"
  [[ -d "/sys/class/net/${interface}/wireless" ]] && return 0
  [[ -L "/sys/class/net/${interface}/phy80211" ]] && return 0
  return 1
}

go2_first_wireless_interface() {
  local path
  for path in /sys/class/net/*; do
    [[ -e "${path}" ]] || continue
    if go2_is_wireless_interface "$(basename "${path}")"; then
      basename "${path}"
      return 0
    fi
  done
  return 1
}

go2_interface_ipv4() {
  local interface="$1"
  ip -4 -o addr show dev "${interface}" scope global 2>/dev/null \
    | awk '{print $4}' \
    | paste -sd, -
}

go2_interface_operstate() {
  local interface="$1"
  cat "/sys/class/net/${interface}/operstate" 2>/dev/null || printf 'unknown\n'
}

go2_interface_carrier() {
  local interface="$1"
  cat "/sys/class/net/${interface}/carrier" 2>/dev/null || printf 'unknown\n'
}

go2_interface_has_multicast() {
  local interface="$1"
  local flags
  flags="$(cat "/sys/class/net/${interface}/flags" 2>/dev/null || true)"
  [[ -n "${flags}" ]] || return 1
  (( (flags & 0x1000) != 0 ))
}

go2_interface_ready() {
  local interface="$1"
  [[ -d "/sys/class/net/${interface}" ]] || return 1
  [[ "$(go2_interface_carrier "${interface}")" != "0" ]] || return 1
  [[ -n "$(go2_interface_ipv4 "${interface}")" ]] || return 1
  go2_interface_has_multicast "${interface}"
}

go2_resolve_network_interface() {
  local selector="${1:-ethernet}"
  local explicit_interface="${2:-}"
  local interface=""

  if [[ -n "${explicit_interface}" ]]; then
    printf '%s\n' "${explicit_interface}"
    return 0
  fi

  case "${selector}" in
    ethernet)
      interface="${GO2_ETH_IF_DEFAULT}"
      ;;
    wifi|wireless)
      interface="${GO2_WIFI_IF_DEFAULT}"
      if [[ -z "${interface}" ]]; then
        interface="$(go2_first_wireless_interface || true)"
      fi
      ;;
    auto)
      if [[ -n "${GO2_NET_IF:-}" ]] && go2_interface_ready "${GO2_NET_IF}"; then
        interface="${GO2_NET_IF}"
      elif go2_interface_ready "${GO2_ETH_IF_DEFAULT}"; then
        interface="${GO2_ETH_IF_DEFAULT}"
      else
        local wifi_candidate="${GO2_WIFI_IF_DEFAULT}"
        if [[ -z "${wifi_candidate}" ]]; then
          wifi_candidate="$(go2_first_wireless_interface || true)"
        fi
        if [[ -n "${wifi_candidate}" ]] && go2_interface_ready "${wifi_candidate}"; then
          interface="${wifi_candidate}"
        else
          echo "No ready Go2 transport found." >&2
          echo "Ethernet candidate: ${GO2_ETH_IF_DEFAULT}" >&2
          echo "Wi-Fi candidate: ${wifi_candidate:-<none>}" >&2
          echo "Inspect links with:" >&2
          echo "  bash scripts/deploy/run_unitree_mjlab_sim_deploy.sh network-status ethernet" >&2
          echo "  bash scripts/deploy/run_unitree_mjlab_sim_deploy.sh network-status wifi" >&2
          return 2
        fi
      fi
      ;;
    *)
      # Backward-compatible direct interface form.
      interface="${selector}"
      ;;
  esac

  if [[ -z "${interface}" ]]; then
    echo "No interface found for transport '${selector}'." >&2
    if [[ "${selector}" == "wifi" || "${selector}" == "wireless" ]]; then
      echo "Set GO2_WIFI_IF or connect a wireless adapter." >&2
    fi
    return 2
  fi
  printf '%s\n' "${interface}"
}

go2_print_network_status() {
  local interface="$1"
  if [[ ! -d "/sys/class/net/${interface}" ]]; then
    printf 'interface=%s exists=no\n' "${interface}"
    return 1
  fi

  local transport="ethernet"
  if go2_is_wireless_interface "${interface}"; then
    transport="wifi"
  fi

  printf 'interface=%s\n' "${interface}"
  printf 'transport=%s\n' "${transport}"
  printf 'operstate=%s\n' "$(go2_interface_operstate "${interface}")"
  printf 'carrier=%s\n' "$(go2_interface_carrier "${interface}")"
  printf 'ipv4=%s\n' "$(go2_interface_ipv4 "${interface}")"
  if go2_interface_has_multicast "${interface}"; then
    printf 'multicast=yes\n'
  else
    printf 'multicast=no\n'
  fi
  if command -v nmcli >/dev/null 2>&1; then
    nmcli -t -f GENERAL.STATE,GENERAL.CONNECTION device show "${interface}" 2>/dev/null || true
    if go2_is_wireless_interface "${interface}"; then
      local active_ssid
      active_ssid="$(
        nmcli -t -f IN-USE,SSID device wifi list ifname "${interface}" --rescan no 2>/dev/null \
          | sed -n -e 's/^yes://p' -e 's/^\*://p' \
          | head -n 1
      )"
      printf 'ssid=%s\n' "${active_ssid}"
    fi
  fi
}

go2_active_wifi_ssid() {
  local interface="$1"
  command -v nmcli >/dev/null 2>&1 || return 1
  nmcli -t -f IN-USE,SSID device wifi list ifname "${interface}" --rescan no 2>/dev/null \
    | sed -n -e 's/^yes://p' -e 's/^\*://p' \
    | head -n 1
}

go2_validate_network_interface() {
  local interface="$1"

  if [[ ! -d "/sys/class/net/${interface}" ]]; then
    echo "Network interface '${interface}' does not exist." >&2
    echo "Available interfaces:" >&2
    ls -1 /sys/class/net >&2
    return 2
  fi

  local carrier
  carrier="$(go2_interface_carrier "${interface}")"
  if [[ "${carrier}" == "0" ]]; then
    echo "Network interface '${interface}' has no link." >&2
    if go2_is_wireless_interface "${interface}"; then
      echo "Connect it to the Go2 dongle's WLAN first:" >&2
      echo "  bash scripts/deploy/run_unitree_mjlab_sim_deploy.sh wifi-connect <SSID> ${interface}" >&2
    fi
    return 2
  fi

  local ipv4
  ipv4="$(go2_interface_ipv4 "${interface}")"
  if [[ -z "${ipv4}" ]]; then
    echo "Network interface '${interface}' has no global IPv4 address." >&2
    echo "CycloneDDS cannot bind to it in this state." >&2
    return 2
  fi

  if ! go2_interface_has_multicast "${interface}"; then
    echo "Network interface '${interface}' is not multicast-capable." >&2
    echo "Unitree DDS discovery requires multicast on the selected link." >&2
    return 2
  fi

  if go2_is_wireless_interface "${interface}"; then
    local ssid
    ssid="$(go2_active_wifi_ssid "${interface}" || true)"
    echo "[NETWORK] transport=wifi interface=${interface} ipv4=${ipv4}"
    echo "[NETWORK] ssid=${ssid:-<unknown>}"
    echo "[NETWORK] Wi-Fi must be on the same LAN as the Go2 dongle, with client isolation disabled."
    if [[ "${ssid,,}" == *guest* ]]; then
      echo "[WARN] SSID '${ssid}' appears to be a guest network." >&2
      echo "[WARN] Guest WLANs commonly block multicast and client-to-client traffic required by Unitree DDS." >&2
    fi
  else
    echo "[NETWORK] transport=ethernet interface=${interface} ipv4=${ipv4}"
  fi
}
