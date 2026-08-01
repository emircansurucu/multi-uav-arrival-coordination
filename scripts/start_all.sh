#!/usr/bin/env bash
# Sistemi tek komutla ayaga kaldirir: uc SITL + uc XRCE agent, ardindan
# uc arac agent node'u.
#
# Kullanim: scripts/start_all.sh [hiz_carpani] [ruzgar_parm_dosyasi]
# Hiz carpani yalnizca gelistirme testleri icindir; zamanlama olcumleri 1.0
# ile yapilmalidir.
# Ruzgar senaryolari icin ornek:
#   scripts/start_all.sh 1 ros2_ws/src/oasy_bringup/params/wind/steady.parm
set -euo pipefail

SPEEDUP="${1:-1}"
EXTRA_PARM="${2:-}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${PROJECT_DIR}/logs/run_$(date +%Y%m%d_%H%M%S)"
VEHICLES=(1 2 3)
DDS_READY_ATTEMPTS=40
DDS_READY_INTERVAL_S=3

mkdir -p "${LOG_DIR}"
echo "loglar: ${LOG_DIR}"

cleanup() {
  echo
  echo "kapatiliyor..."
  bash "${PROJECT_DIR}/scripts/stop_all.sh"
}
trap cleanup EXIT INT TERM

for vehicle in "${VEHICLES[@]}"; do
  echo "[HA-${vehicle}] SITL baslatiliyor"
  bash "${PROJECT_DIR}/scripts/start_sitl.sh" "${vehicle}" "${SPEEDUP}" "${EXTRA_PARM}" \
    > "${LOG_DIR}/sitl_ha${vehicle}.log" 2>&1 &
done

# DDS oturumunun kuruldugunu agent logundan dogruluyoruz; ros2 CLI
# AP_DDS konularini guvenilir sekilde gostermiyor.
for vehicle in "${VEHICLES[@]}"; do
  ready=0
  for _ in $(seq 1 "${DDS_READY_ATTEMPTS}"); do
    if grep -q "create_datawriter" "${LOG_DIR}/sitl_ha${vehicle}.log" 2>/dev/null; then
      ready=1
      break
    fi
    sleep "${DDS_READY_INTERVAL_S}"
  done
  if [[ "${ready}" -ne 1 ]]; then
    echo "HATA: HA-${vehicle} DDS oturumu kurulamadi, ${LOG_DIR}/sitl_ha${vehicle}.log kontrol edin" >&2
    exit 1
  fi
  echo "[HA-${vehicle}] DDS hazir"
done

echo "uc agent node baslatiliyor"
ros2 launch oasy_bringup three_vehicles.launch.py 2>&1 | tee "${LOG_DIR}/agents.log"
