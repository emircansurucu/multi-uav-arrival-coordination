#!/usr/bin/env bash
# Tek bir HA icin XRCE agent'i ve ArduPlane SITL'ini baslatir.
# Kullanim: scripts/start_sitl.sh <1|2|3> [hiz_carpani]
# Hiz carpani yalnizca gelistirme testlerini kisaltmak icindir; zamanlama
# olcumleri 1.0 ile yapilmalidir.
set -euo pipefail

VEHICLE="${1:-}"
SPEEDUP="${2:-1}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARAM_DIR="${PROJECT_DIR}/ros2_ws/src/oasy_bringup/params"

# Kalkis koordinatlari vaka dokumani Tablo 1'den. Kalkis yonu dokumanda
# verilmedigi icin her arac ilk rota noktasina dogru yonlendirilmistir.
# Home irtifasi ucunde 0 alinarak MSL/relatif fark degiskeni ortadan kaldirilir.
case "${VEHICLE}" in
  1) INSTANCE=0; UDP_PORT=2019; LOCATION="47.530002,-122.302457,0,11" ;;
  2) INSTANCE=1; UDP_PORT=2020; LOCATION="47.451146,-122.317983,0,29" ;;
  3) INSTANCE=2; UDP_PORT=2021; LOCATION="47.492515,-122.215659,0,30" ;;
  *) echo "Kullanim: $0 <1|2|3>" >&2; exit 1 ;;
esac

PARAM_FILE="${PARAM_DIR}/ha${VEHICLE}.parm"
if [[ ! -f "${PARAM_FILE}" ]]; then
  echo "Parametre dosyasi bulunamadi: ${PARAM_FILE}" >&2
  exit 1
fi

# Agent'i once baslatiyoruz; DDS istemcisi acilista agent'i bulamazsa
# DDS_MAX_RETRY denemesinden sonra vazgeciyor.
echo "[HA-${VEHICLE}] XRCE agent baslatiliyor (udp4 port ${UDP_PORT})"
ros2 run micro_ros_agent micro_ros_agent udp4 -p "${UDP_PORT}" &
AGENT_PID=$!
trap 'echo "[HA-'"${VEHICLE}"'] agent kapatiliyor"; kill "${AGENT_PID}" 2>/dev/null || true' EXIT
sleep 2

# -w: eeprom silinir, boylece her kosu ayni parametre durumundan baslar.
# -N: binary yeniden derlenmez.
# --no-mavproxy: koordinasyon MAVProxy uzerinden yurumeyecek, SITL'e
#   pymavlink ile dogrudan tcp:127.0.0.1:$((5760 + 10 * INSTANCE)) baglanilir.
echo "[HA-${VEHICLE}] SITL baslatiliyor (instance ${INSTANCE}, konum ${LOCATION}, hiz ${SPEEDUP}x)"
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
  --add-param-file="${PARAM_FILE}"
