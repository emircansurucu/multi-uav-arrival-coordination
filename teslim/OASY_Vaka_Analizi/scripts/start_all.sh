#!/usr/bin/env bash
# üç sitl xrce agent ve araç düğümünü başlatır
# kullanım scripts/start_all.sh [hız_çarpanı] [rüzgâr_parametre_dosyası]
set -euo pipefail

SPEEDUP="${1:-1}"  # simülasyon hız çarpanı
EXTRA_PARM="${2:-}"  # ek parametre dosyası
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"  # proje dizini
LOG_DIR="${PROJECT_DIR}/logs/run_$(date +%Y%m%d_%H%M%S)"  # koşu kayıt dizini
VEHICLES=(1 2 3)  # başlatılacak araçlar
DDS_READY_ATTEMPTS=40  # dds hazırlık deneme sayısı
DDS_READY_INTERVAL_S=3  # dds denemeleri arasındaki süre

mkdir -p "${LOG_DIR}"
echo "loglar: ${LOG_DIR}"

cleanup() {
  echo
  echo "kapatılıyor"
  bash "${PROJECT_DIR}/scripts/stop_all.sh"
}
trap cleanup EXIT INT TERM

for vehicle in "${VEHICLES[@]}"; do
  echo "[HA-${vehicle}] SITL başlatılıyor"
  bash "${PROJECT_DIR}/scripts/start_sitl.sh" "${vehicle}" "${SPEEDUP}" "${EXTRA_PARM}" \
    > "${LOG_DIR}/sitl_ha${vehicle}.log" 2>&1 &
done

# dds hazırlığını sitl günlüğünden denetler
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
    echo "HATA: HA-${vehicle} DDS oturumu kurulamadı, ${LOG_DIR}/sitl_ha${vehicle}.log kontrol edin" >&2
    exit 1
  fi
  echo "[HA-${vehicle}] DDS hazır"
done

echo "üç araç düğümü başlatılıyor"
ros2 launch oasy_bringup three_vehicles.launch.py 2>&1 | tee "${LOG_DIR}/agents.log"
