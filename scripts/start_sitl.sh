#!/usr/bin/env bash
# tek bir araç için xrce agent ve arduplane sitl başlatır
# kullanım scripts/start_sitl.sh <1|2|3> [hız_çarpanı] [ek_parametre_dosyası]
set -euo pipefail

VEHICLE="${1:-}"  # araç kimliği
SPEEDUP="${2:-1}"  # simülasyon hız çarpanı
EXTRA_PARM="${3:-}"  # ek parametre dosyası
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"  # proje dizini
PARAM_DIR="${PROJECT_DIR}/ros2_ws/src/oasy_bringup/params"  # araç parametre dizini

# araç örneği portu başlangıç konumu ve kalkış yönü
case "${VEHICLE}" in
  1) INSTANCE=0; UDP_PORT=2019; LOCATION="47.530002,-122.302457,0,11" ;;
  2) INSTANCE=1; UDP_PORT=2020; LOCATION="47.451146,-122.317983,0,29" ;;
  3) INSTANCE=2; UDP_PORT=2021; LOCATION="47.492515,-122.215659,0,30" ;;
  *) echo "Kullanım: $0 <1|2|3>" >&2; exit 1 ;;
esac

PARAM_FILE="${PARAM_DIR}/ha${VEHICLE}.parm"
if [[ ! -f "${PARAM_FILE}" ]]; then
  echo "Parametre dosyası bulunamadı: ${PARAM_FILE}" >&2
  exit 1
fi

EFFECTIVE_PARM="${PARAM_FILE}"
if [[ -n "${EXTRA_PARM}" ]]; then
  # ek parametre dosyasını mutlak yola çevirir
  EXTRA_PARM_ABS="$(readlink -f "${EXTRA_PARM}" 2>/dev/null || true)"
  if [[ -z "${EXTRA_PARM_ABS}" || ! -f "${EXTRA_PARM_ABS}" ]]; then
    echo "Ek parametre dosyası bulunamadı: ${EXTRA_PARM}" >&2
    exit 1
  fi
  # araç ve senaryo parametrelerini tek dosyada birleştirir
  mkdir -p "${PROJECT_DIR}/logs"
  EFFECTIVE_PARM="${PROJECT_DIR}/logs/effective_ha${VEHICLE}.parm"
  cat "${PARAM_FILE}" "${EXTRA_PARM_ABS}" > "${EFFECTIVE_PARM}"
  echo "[HA-${VEHICLE}] ek parametreler: ${EXTRA_PARM_ABS}"
fi

# xrce agent dds istemcisinden önce başlatılır
echo "[HA-${VEHICLE}] XRCE agent başlatılıyor (udp4 port ${UDP_PORT})"
ros2 run micro_ros_agent micro_ros_agent udp4 -p "${UDP_PORT}" &
AGENT_PID=$!
trap 'echo "[HA-'"${VEHICLE}"'] agent kapatılıyor"; kill "${AGENT_PID}" 2>/dev/null || true' EXIT
sleep 2

# sitl temiz parametrelerle mavproxy olmadan başlatılır
echo "[HA-${VEHICLE}] SITL başlatılıyor (örnek ${INSTANCE}, konum ${LOCATION}, hız ${SPEEDUP}x)"
exec sim_vehicle.py \
  -v ArduPlane \
  -f plane \
  -I "${INSTANCE}" \
  -w \
  -N \
  -S "${SPEEDUP}" \
  --no-mavproxy \
  --enable-dds \
  -l "${LOCATION}" \
  --add-param-file="${EFFECTIVE_PARM}" \
  -A "--serial0=tcp:0 --serial1=tcp:2"
