#!/usr/bin/env bash
# ortamı hazırlar ve seçilen uçuş senaryosunu çalıştırır
# kullanım ./run.sh [calm|steady|variable|extreme|all|--check]
set -uo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"  # proje dizini
ROS_DIST="${ROS_DISTRO:-humble}"  # kullanılacak ros dağıtımı
WIND_DIR="ros2_ws/src/oasy_bringup/params/wind"  # rüzgâr parametre dizini

log_error()  { printf '\033[31m%s\033[0m\n' "$*"; }
log_ok()    { printf '\033[32m%s\033[0m\n' "$*"; }
log_info() { printf '\033[36m%s\033[0m\n' "$*"; }

# ön koşullar

missing=0

check_requirement() {
  local name="$1" cond="$2" hint="$3"
  if eval "${cond}" >/dev/null 2>&1; then
    log_ok "  [ok]  ${name}"
  else
    log_error "  [yok] ${name}"
    echo "        ${hint}"
    missing=$((missing + 1))
  fi
}

echo "ön koşullar denetleniyor"

check_requirement "ROS 2 ${ROS_DIST}" \
  "[ -f /opt/ros/${ROS_DIST}/setup.bash ]" \
  "kurulum: https://docs.ros.org/en/humble/Installation.html"

check_requirement "Python 3.10+" \
  "python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)'" \
  "belge Python 3.10.12 istiyor"

check_requirement "pymavlink" \
  "python3 -c 'import pymavlink'" \
  "pip install -r ${PROJECT_DIR}/requirements.txt"

check_requirement "geographiclib" \
  "python3 -c 'import geographiclib'" \
  "pip install -r ${PROJECT_DIR}/requirements.txt"

# sistemde yoksa proje içindeki sitl aracını kullanır
SITL_TOOL="$(command -v sim_vehicle.py || true)"
if [ -z "${SITL_TOOL}" ] && [ -x "${PROJECT_DIR}/ardupilot/Tools/autotest/sim_vehicle.py" ]; then
  export PATH="${PROJECT_DIR}/ardupilot/Tools/autotest:${PATH}"
  SITL_TOOL="${PROJECT_DIR}/ardupilot/Tools/autotest/sim_vehicle.py"
fi

check_requirement "ArduPlane SITL (sim_vehicle.py)" \
  "[ -n \"${SITL_TOOL}\" ]" \
  "ArduPilot kaynağını derleyin: cd ardupilot && ./waf configure --board sitl && ./waf plane"

check_requirement "ArduPlane SITL binary (derlenmiş)" \
  "[ -x ${PROJECT_DIR}/ardupilot/build/sitl/bin/arduplane ] || command -v arduplane" \
  "cd ardupilot && ./waf configure --board sitl && ./waf plane"

check_requirement "micro_ros_agent" \
  "[ -d ${HOME}/ardu_ws/install/micro_ros_agent ]" \
  "AP_DDS köprüsü için micro-ROS agent gerekli"

if [ "${missing}" -gt 0 ]; then
  echo
  log_error "${missing} ön koşul sağlanmadı, ayrıntılar için README.md"
  exit 1
fi

if [ "${1:-}" = "--check" ]; then
  echo
  log_ok "bütün ön koşullar sağlandı"
  exit 0
fi

# ortam

# ros kurulumu için tanımsız değişken denetimini geçici kapatır
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DIST}/setup.bash"
if [ -f "${PROJECT_DIR}/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
else
  set -u
  log_error "çalışma alanı derlenmemiş"
  echo "  cd ${PROJECT_DIR}/ros2_ws && colcon build"
  exit 1
fi
set -u

# senaryo seçimi

SCENARIO="${1:-calm}"
WIND_PARM=""
WIND_PROFILE=""

case "${SCENARIO}" in
  calm|sakin|"")
    LABEL="sakin hava" ;;
  steady|sabit)
    LABEL="sabit 8 m/s rüzgâr"
    WIND_PARM="${WIND_DIR}/steady.parm" ;;
  variable|degisken)
    LABEL="değişken rüzgâr (cephe geçişi, 0.50 derece/s)"
    WIND_PARM="${WIND_DIR}/variable.parm"
    WIND_PROFILE="normal" ;;
  extreme|uc)
    LABEL="uç durum (fırtına çıkış cephesi, 5.5 derece/s)"
    WIND_PARM="${WIND_DIR}/variable.parm"
    WIND_PROFILE="extreme" ;;
  all|hepsi)
    LABEL="üç senaryo sırayla" ;;
  *)
    log_error "bilinmeyen senaryo: ${SCENARIO}"
    echo "  geçerli: calm, steady, variable, extreme, all"
    exit 1 ;;
esac

echo
log_info "senaryo: ${LABEL}"

cd "${PROJECT_DIR}"

# tek senaryoyu çalıştırıp analiz sonucunu döndürür
run_scenario() {
  local name="$1" parm="$2" profile="$3"
  local log_file
  log_file="$(mktemp)"

  # önceki koşudan kalan süreçleri temizler
  bash "${PROJECT_DIR}/scripts/stop_all.sh" >/dev/null 2>&1 || true
  pgrep -f 'wind_prof[i]le' | xargs -r kill 2>/dev/null || true
  sleep 5

  if [ -n "${parm}" ]; then
    bash scripts/start_all.sh 1 "${parm}" > "${log_file}" 2>&1 &
  else
    bash scripts/start_all.sh 1 > "${log_file}" 2>&1 &
  fi
  local launcher_pid=$!

  # üç dds oturumunun kurulmasını bekler
  local ready=0 i
  for i in $(seq 1 90); do
    if [ "$(grep -c 'DDS hazır' "${log_file}" 2>/dev/null || true)" -ge 3 ] 2>/dev/null; then
      ready=1
      break
    fi
    if grep -q 'HATA:' "${log_file}" 2>/dev/null; then
      break
    fi
    sleep 3
  done
  if [ "${ready}" -ne 1 ]; then
    log_error "  ${name}: SITL ayağa kalkmadı"
    kill "${launcher_pid}" 2>/dev/null || true
    bash "${PROJECT_DIR}/scripts/stop_all.sh" >/dev/null 2>&1 || true
    return 2
  fi

  if [ -n "${profile}" ]; then
    sleep 20
    if [ "${profile}" = "extreme" ]; then
      python3 scripts/wind_profile.py --extreme >/dev/null 2>&1 &
    else
      python3 scripts/wind_profile.py >/dev/null 2>&1 &
    fi
  fi

  # üç aracın varmasını bekler
  local arrived=0
  for i in $(seq 1 120); do
    if [ "$(grep -c 'durum: ARRIVED -> RTL' "${log_file}" 2>/dev/null || true)" -ge 3 ] 2>/dev/null; then
      arrived=1
      break
    fi
    sleep 10
  done

  local run_dir
  run_dir="$(ls -dt "${PROJECT_DIR}"/logs/run_* 2>/dev/null | head -1)"

  if [ "${arrived}" -ne 1 ]; then
    log_error "  ${name}: üç araç varmadı (zaman aşımı)"
    bash "${PROJECT_DIR}/scripts/stop_all.sh" >/dev/null 2>&1 || true
    pgrep -f 'wind_prof[i]le' | xargs -r kill 2>/dev/null || true
    rm -f "${log_file}"
    return 2
  fi

  # analiz çalışırken araç yayınlarını açık tutar
  local sonuc
  python3 scripts/analyze_run.py --timeout 60 --log "${run_dir}/agents.log"
  sonuc=$?

  bash "${PROJECT_DIR}/scripts/stop_all.sh" >/dev/null 2>&1 || true
  pgrep -f 'wind_prof[i]le' | xargs -r kill 2>/dev/null || true
  rm -f "${log_file}"
  return "${sonuc}"
}

if [ "${SCENARIO}" = "all" ] || [ "${SCENARIO}" = "hepsi" ]; then
  echo "üç senaryo sırayla koşulacak; her biri ~11 dakika"
  echo
  results=()
  for spec in "sakin::" "sabit:${WIND_DIR}/steady.parm:" "değişken:${WIND_DIR}/variable.parm:normal"; do
    name="${spec%%:*}"
    rest="${spec#*:}"
    parm="${rest%%:*}"
    profile="${rest#*:}"

    echo "=============================================================="
    log_info "SENARYO: ${name}"
    echo "=============================================================="
    if run_scenario "${name}" "${parm}" "${profile}"; then
      results+=("${name}:OK")
      log_ok "  -> ${name}: GEÇTİ"
    else
      results+=("${name}:FAIL")
      log_error "  -> ${name}: KALDI"
    fi
    echo
  done

  echo "=============================================================="
  log_info "ÖZET"
  echo "=============================================================="
  passed=0
  for r in "${results[@]}"; do
    name="${r%%:*}"; status="${r##*:}"
    if [ "${status}" = "OK" ]; then
      log_ok "  ${name}: GEÇTİ"
      passed=$((passed + 1))
    else
      log_error "  ${name}: KALDI"
    fi
  done
  echo
  echo "  ${passed}/3 senaryo geçti"
  [ "${passed}" -eq 3 ] || exit 1
  exit 0
fi

# önceki koşudan kalan süreçleri temizler
bash "${PROJECT_DIR}/scripts/stop_all.sh" >/dev/null 2>&1 || true
sleep 3

if [ -n "${WIND_PROFILE}" ]; then
  # rüzgâr profilini sitl başladıktan sonra uygular
  (
    for _ in $(seq 1 60); do
      if pgrep -f 'sim_vehic[l]e.py' >/dev/null 2>&1; then
        sleep 25
        if [ "${WIND_PROFILE}" = "extreme" ]; then
          python3 scripts/wind_profile.py --extreme
        else
          python3 scripts/wind_profile.py
        fi
        break
      fi
      sleep 2
    done
  ) &
fi

if [ -n "${WIND_PARM}" ]; then
  exec bash scripts/start_all.sh 1 "${WIND_PARM}"
else
  exec bash scripts/start_all.sh 1
fi
