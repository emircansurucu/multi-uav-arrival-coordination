#!/usr/bin/env bash
# Tek bir HA icin XRCE agent'i ve ArduPlane SITL'ini baslatir.
# Kullanim: scripts/start_sitl.sh <1|2|3> [hiz_carpani] [ek_parm_dosyasi]
# Hiz carpani yalnizca gelistirme testlerini kisaltmak icindir; zamanlama
# olcumleri 1.0 ile yapilmalidir.
# Ek parm dosyasi ruzgar senaryolari icin kullanilir (params/wind/*.parm).
set -euo pipefail

VEHICLE="${1:-}"
SPEEDUP="${2:-1}"
EXTRA_PARM="${3:-}"
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

EFFECTIVE_PARM="${PARAM_FILE}"
if [[ -n "${EXTRA_PARM}" ]]; then
  # sim_vehicle.py binary'yi kendi calisma dizininden baslatiyor; goreli yol
  # orada cozulmez.
  EXTRA_PARM_ABS="$(readlink -f "${EXTRA_PARM}" 2>/dev/null || true)"
  if [[ -z "${EXTRA_PARM_ABS}" || ! -f "${EXTRA_PARM_ABS}" ]]; then
    echo "Ek parametre dosyasi bulunamadi: ${EXTRA_PARM}" >&2
    exit 1
  fi
  # Iki dosya ayri ayri --add-param-file ile verildiginde arac parametreleri
  # sessizce uygulanmiyor (DDS_DOMAIN_ID varsayilanda kaliyor ve arac yanlis
  # domain'e baglaniyor). Tek dosyada birlestirmek bunu onluyor.
  mkdir -p "${PROJECT_DIR}/logs"
  EFFECTIVE_PARM="${PROJECT_DIR}/logs/effective_ha${VEHICLE}.parm"
  cat "${PARAM_FILE}" "${EXTRA_PARM_ABS}" > "${EFFECTIVE_PARM}"
  echo "[HA-${VEHICLE}] ek parametreler: ${EXTRA_PARM_ABS}"
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
# --serial0=tcp:0: SERIAL0 varsayilani "tcp:5760:wait" oldugu icin SITL,
#   bir GCS baglanana kadar accept() uzerinde bloklar ve ana dongu -
#   dolayisiyla DDS thread'i - hic calismaz. MAVProxy kullanmadigimiz icin
#   bekleme kaldirilir; "tcp:0" instance port ofsetini korur.
# --serial1=tcp:2: ucus sirasinda ruzgar parametrelerini degistirmek icin
#   ikinci bir baglanti noktasi. SERIAL0'i agent kullaniyor ve SITL'in tcp
#   portu tek istemci kabul ediyor. Port = 5760 + 10*instance + 2
#   (UARTDriver.cpp: port <= 1000 ise base_port() + port).
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
  --add-param-file="${EFFECTIVE_PARM}" \
  -A "--serial0=tcp:0 --serial1=tcp:2"
