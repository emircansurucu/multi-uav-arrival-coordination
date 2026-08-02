#!/usr/bin/env bash
# Senaryo matrisini ardisik kosar ve her kosunun raporunu toplar.
#
# Tek kosu n=1'dir ve bu projede birkac kez yaniltici cikti; sart
# saglandiginin soylenebilmesi icin her senaryo tekrarlanir.
#
# Kullanim: scripts/run_matrix.sh [tekrar_sayisi]
set -uo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPEATS="${1:-2}"
WIND_DIR="ros2_ws/src/oasy_bringup/params/wind"
MATRIX_DIR="${PROJECT_DIR}/logs/matrix_$(date +%Y%m%d_%H%M%S)"
# start_all.sh 'ros2 launch oasy_bringup' cagiriyor; calisma alani burada
# source edilmezse paket bulunamiyor ve kosu sessizce bos gecmis oluyor.
# ROS setup betikleri tanimsiz degisken okuyor, bu yuzden -u gecici kapali.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
set -u
SUMMARY="${MATRIX_DIR}/ozet.txt"
# Kalkis + ~200 s tirmanis + ~600 s seyir; ustune genis pay.
ARRIVAL_TIMEOUT_S=1500
POLL_INTERVAL_S=10

# senaryo_adi:ruzgar_parametre_dosyasi (bos = sakin hava)
SCENARIOS=(
  "sakin:"
  "sabit8:${WIND_DIR}/steady.parm"
  "gusty12:${WIND_DIR}/gusty.parm"
)

mkdir -p "${MATRIX_DIR}"
echo "matris dizini: ${MATRIX_DIR}"
{
  echo "OASY senaryo matrisi - $(date '+%Y-%m-%d %H:%M')"
  echo "her senaryo ${REPEATS} kez kosuluyor"
  echo
} > "${SUMMARY}"

run_once() {
  local ad="$1" parm="$2" tekrar="$3"
  local etiket="${ad}_${tekrar}"
  echo "=== ${etiket} basliyor ($(date '+%H:%M:%S')) ==="

  local onceki_dir
  onceki_dir="$(ls -td "${PROJECT_DIR}"/logs/run_* 2>/dev/null | head -1 || true)"

  bash "${PROJECT_DIR}/scripts/start_all.sh" 1 "${parm}" \
    > "${MATRIX_DIR}/${etiket}_launch.log" 2>&1 &
  local launch_pid=$!

  # Yeni kosu dizininin olusmasini bekle.
  local run_dir="" bekleme=0
  while [[ ${bekleme} -lt 120 ]]; do
    run_dir="$(ls -td "${PROJECT_DIR}"/logs/run_* 2>/dev/null | head -1 || true)"
    [[ -n "${run_dir}" && "${run_dir}" != "${onceki_dir}" && -f "${run_dir}/agents.log" ]] && break
    sleep 2
    bekleme=$((bekleme + 2))
    run_dir=""
  done
  if [[ -z "${run_dir}" ]]; then
    echo "${etiket}: BASLATILAMADI" | tee -a "${SUMMARY}"
    kill "${launch_pid}" 2>/dev/null
    wait "${launch_pid}" 2>/dev/null
    return 1
  fi

  # Uc varis gorunene kadar bekle.
  local gecen=0 varis=0
  while [[ ${gecen} -lt ${ARRIVAL_TIMEOUT_S} ]]; do
    varis="$(grep -c "HEDEFE VARILDI" "${run_dir}/agents.log" 2>/dev/null)" || true
    [[ -z "${varis}" ]] && varis=0
    [[ "${varis}" -ge 3 ]] && break
    sleep "${POLL_INTERVAL_S}"
    gecen=$((gecen + POLL_INTERVAL_S))
  done

  if [[ "${varis}" -ge 3 ]]; then
    # Analizor canli dinleyici; araclar RTL'de hala yayinda oldugu icin
    # varislardan hemen sonra calistirilmali.
    bash -c "source /opt/ros/humble/setup.bash >/dev/null 2>&1
             source '${PROJECT_DIR}/ros2_ws/install/setup.bash' >/dev/null 2>&1
             python3 '${PROJECT_DIR}/scripts/analyze_run.py' --timeout 30 \
               --output '${MATRIX_DIR}/${etiket}.json'" \
      > "${MATRIX_DIR}/${etiket}_analiz.log" 2>&1
    grep -E "HA-[0-9]:|HA-[0-9] -|varis sirasi" "${MATRIX_DIR}/${etiket}_analiz.log" \
      | sed "s/^/${etiket}  /" >> "${SUMMARY}"
  else
    echo "${etiket}: ZAMAN ASIMI (${varis}/3 varis)" >> "${SUMMARY}"
  fi
  echo "${etiket}  kosu: ${run_dir}" >> "${SUMMARY}"
  echo >> "${SUMMARY}"

  # Once stop_all cagrilir: start_all.sh son satirinda 'ros2 launch | tee'
  # calistiriyor ve bash on plandaki komut bitene kadar TERM trap'ini
  # islemiyor. Dogrudan kill edilirse trap hic atesienmiyor ve wait sonsuza
  # kadar blokluyor (olculen: kosu bitmis olmasina ragmen 37 dakika bekledi).
  # stop_all desenle three_vehicles.launch.py'yi de oldurdugu icin boru
  # hatti kapanir ve start_all kendiliginden cikar.
  bash "${PROJECT_DIR}/scripts/stop_all.sh" >/dev/null 2>&1
  kill "${launch_pid}" 2>/dev/null
  local bekle=0
  while kill -0 "${launch_pid}" 2>/dev/null && [[ ${bekle} -lt 60 ]]; do
    sleep 1
    bekle=$((bekle + 1))
  done
  kill -9 "${launch_pid}" 2>/dev/null
  wait "${launch_pid}" 2>/dev/null
  echo "=== ${etiket} bitti ($(date '+%H:%M:%S')) ==="
}

for senaryo in "${SCENARIOS[@]}"; do
  ad="${senaryo%%:*}"
  parm="${senaryo#*:}"
  for tekrar in $(seq 1 "${REPEATS}"); do
    run_once "${ad}" "${parm}" "${tekrar}"
  done
done

echo
echo "matris tamamlandi. ozet: ${SUMMARY}"
cat "${SUMMARY}"
