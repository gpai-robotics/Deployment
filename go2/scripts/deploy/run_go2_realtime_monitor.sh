#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NET_IF="${1:-enp0s31f6}"
HISTORY_SEC="${2:-20}"
SAMPLE_HZ="${3:-25}"
LABEL="${4:-asymppo}"
GO2_HW_PYTHON="${GO2_HW_PYTHON:-python3}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${ROOT_DIR}/artifacts/go2_realtime_monitor"
JSONL_OUT="${OUT_DIR}/${STAMP}_${LABEL}_monitor.jsonl"
source "${ROOT_DIR}/scripts/deploy/go2_network.sh"

cd "${ROOT_DIR}"
mkdir -p "${OUT_DIR}"
if [[ ! -x "${GO2_HW_PYTHON}" ]]; then
  echo "Missing hardware Python: ${GO2_HW_PYTHON}" >&2
  exit 2
fi
go2_validate_network_interface "${NET_IF}"
echo "[INFO] Saving live monitor samples to ${JSONL_OUT}"
echo "[SAFETY] Read-only DDS subscriber: this process does not publish LowCmd."
exec "${GO2_HW_PYTHON}" scripts/deploy/monitor_go2_realtime.py \
  --net-if "${NET_IF}" \
  --history-sec "${HISTORY_SEC}" \
  --sample-hz "${SAMPLE_HZ}" \
  --jsonl-out "${JSONL_OUT}"
