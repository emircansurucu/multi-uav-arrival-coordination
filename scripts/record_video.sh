#!/usr/bin/env bash
# simülasyonun ham ekran kaydını alır
# kullanım scripts/record_video.sh [variable|calm|steady|extreme]
set -uo pipefail

PROJE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.."  # normalize edilmemiş proje yolu
PROJE_DIR="$(cd "${PROJE_DIR}" && pwd)"
: "${DISPLAY:=:1}"
export DISPLAY
KAYIT_FPS=15  # ham video kare hızı
: "${RTL_KUYRUK_S:=360}"  # varıştan sonraki rtl kayıt süresi
export RTL_KUYRUK_S

renk_hata()  { printf '\033[31m%s\033[0m\n' "$*"; }
renk_ok()    { printf '\033[32m%s\033[0m\n' "$*"; }
renk_bilgi() { printf '\033[36m%s\033[0m\n' "$*"; }

# ekran kaydını ve görselleştiriciyi yönetir
SENARYO="${1:-variable}"
case "${SENARYO}" in
  calm|steady|variable|extreme) ;;
  *) renk_hata "bilinmeyen senaryo: ${SENARYO}"; exit 1 ;;
esac

eksik=0
for arac in ffmpeg python3; do
  command -v "${arac}" >/dev/null 2>&1 || { renk_hata "  [yok] ${arac}"; eksik=1; }
done
if ! timeout 5 xdpyinfo >/dev/null 2>&1; then
  renk_hata "  [yok] X ekranı erişilemiyor: DISPLAY=${DISPLAY}"
  eksik=1
fi
[ "${eksik}" -eq 0 ] || { renk_hata "ön koşullar sağlanmadı"; exit 1; }

cd "${PROJE_DIR}"
mkdir -p video logs/video

# görselleştirici için ros ortamını yükler
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
source "${PROJE_DIR}/ros2_ws/install/setup.bash"
set -u

EKRAN="$(xdpyinfo | awk '/dimensions:/ {print $2; exit}')"
KAYIT_G="${EKRAN%x*}"
KAYIT_Y="${EKRAN#*x}"
KAYIT_G=$(( KAYIT_G - KAYIT_G % 2 ))
KAYIT_Y=$(( KAYIT_Y - KAYIT_Y % 2 ))

ZAMAN="$(date +%Y%m%d_%H%M%S)"
HAM_VIDEO="video/ham_${SENARYO}_${ZAMAN}.mp4"
OLAY_LOG="video/olaylar_${SENARYO}_${ZAMAN}.log"
KAYIT_BASLANGIC="$(date +%s)"
printf 'KAYIT_BASLANGIC %s\n' "${KAYIT_BASLANGIC}" > "${OLAY_LOG}"

TEMIZLENDI=0
temizle() {
  [ "${TEMIZLENDI}" -eq 1 ] && return 0
  TEMIZLENDI=1
  [ -n "${OLAY_TAKIP_PID:-}" ] && kill "${OLAY_TAKIP_PID}" 2>/dev/null
  [ -n "${GORSEL_PID:-}" ] && kill "${GORSEL_PID}" 2>/dev/null
  [ -n "${FFMPEG_PID:-}" ] && kill -INT "${FFMPEG_PID}" 2>/dev/null
  bash "${PROJE_DIR}/scripts/stop_all.sh" >/dev/null 2>&1
  pgrep -f 'wind_prof[i]le' | xargs -r kill 2>/dev/null
  return 0
}
# sinyal geldiğinde temizleyip çıkar
trap temizle EXIT
trap 'temizle; exit 143' INT TERM

# önceki koşudan kalan süreçleri temizler
bash scripts/stop_all.sh >/dev/null 2>&1 || true
pgrep -f 'wind_prof[i]le' | xargs -r kill 2>/dev/null || true
sleep 4

renk_bilgi "kayıt başlıyor: ${HAM_VIDEO}  (${KAYIT_G}x${KAYIT_Y} @ ${KAYIT_FPS} fps)"
# ham kayıtta kare kaybını azaltmak için hızlı kodlama kullanır
ffmpeg -loglevel error -y \
  -f x11grab -framerate "${KAYIT_FPS}" -video_size "${KAYIT_G}x${KAYIT_Y}" \
  -draw_mouse 0 -i "${DISPLAY}+0,0" \
  -c:v libx264 -preset ultrafast -crf 20 -pix_fmt yuv420p \
  "${HAM_VIDEO}" >/dev/null 2>&1 &
FFMPEG_PID=$!
renk_bilgi "ekran kaydı etkin, simülasyonu ayrı terminalden başlatabilirsiniz"

# ayrı terminalden başlatılan yeni koşu dizinini bekler
renk_bilgi "simülasyonun başlatılması bekleniyor"
KOSU_DIZIN=""
for _ in $(seq 1 180); do
  aday="$(ls -dt "${PROJE_DIR}"/logs/run_* 2>/dev/null | head -1)"
  if [ -n "${aday}" ] && [ -f "${aday}/agents.log" ]; then
    degisim="$(stat -c %Y "${aday}/agents.log" 2>/dev/null || echo 0)"
    if [ "${degisim}" -ge "${KAYIT_BASLANGIC}" ]; then
      KOSU_DIZIN="${aday}"
      break
    fi
  fi
  sleep 2
done

if [ -z "${KOSU_DIZIN}" ]; then
  renk_hata "yeni simülasyon koşusu bulunamadı"
  exit 1
fi

# montaj için önemli görev olaylarını ayrı günlüğe yazar
stdbuf -oL tail --pid="$$" -n +1 -F "${KOSU_DIZIN}/agents.log" 2>/dev/null \
  | stdbuf -oL grep -E 'durum:|varış planı|HEDEFE VARILDI|KAPI BEKLEMESİ' \
  >> "${OLAY_LOG}" &
OLAY_TAKIP_PID=$!

# araç düğümleri hazır olunca görselleştiriciyi açar
renk_bilgi "araç düğümleri bekleniyor"
hazir=0
for _ in $(seq 1 90); do
  if [ "$(grep -c 'durum: INIT -> CONNECTING' "${KOSU_DIZIN}/agents.log" 2>/dev/null)" -ge 3 ]; then
    hazir=1
    break
  fi
  sleep 3
done

if [ "${hazir}" -eq 1 ]; then
  renk_ok "DDS hazır, görselleştirici açılıyor"
  # görselleştiriciyi terminalin üstünde tam ekran açar
  python3 scripts/visualize.py --fullscreen > logs/video/gorsellestirici.log 2>&1 &
  GORSEL_PID=$!
  sleep 6
  if ! kill -0 "${GORSEL_PID}" 2>/dev/null; then
    renk_hata "görselleştirici açılamadı, ayrıntı: logs/video/gorsellestirici.log"
    tail -5 logs/video/gorsellestirici.log 2>/dev/null
  fi
else
  renk_hata "araç düğümleri başlatılamadı, kayıt yine de sürüyor"
fi

# üç varışı bekler
renk_bilgi "uçuş sürüyor"
vardi=0
for _ in $(seq 1 200); do
  if [ "$(grep -c 'durum: ARRIVED -> RTL' "${KOSU_DIZIN}/agents.log" 2>/dev/null)" -ge 3 ]; then
    vardi=1
    break
  fi
  sleep 8
done

if [ "${vardi}" -eq 1 ]; then
  # analiz araçlar hâlâ yayındayken çalışmalı; kapanış kartı bu çıktıyı kullanır
  renk_ok "varışlar tamam, analiz çalıştırılıyor"
  python3 scripts/analyze_run.py --timeout 60 --log "${KOSU_DIZIN}/agents.log" \
    >> "${OLAY_LOG}" 2>&1

  # araçların kalkış noktalarına dönüşü de kayda girer
  renk_bilgi "RTL fazı: ${RTL_KUYRUK_S} s"
  sleep "${RTL_KUYRUK_S}"

  # görselleştirici kapanınca altındaki terminal yeniden görünür
  [ -n "${GORSEL_PID:-}" ] && kill "${GORSEL_PID}" 2>/dev/null
  GORSEL_PID=""
  sleep 5
else
  renk_hata "zaman aşımı: üç varış tamamlanmadı"
fi

renk_bilgi "kayıt kapatılıyor"
kill -INT "${FFMPEG_PID}" 2>/dev/null
wait "${FFMPEG_PID}" 2>/dev/null
FFMPEG_PID=""

echo
renk_ok "ham kayıt : ${HAM_VIDEO}"
renk_ok "olay logu : ${OLAY_LOG}"
echo
renk_bilgi "altyazılı video hazırlanıyor"
if python3 scripts/edit_video.py "${HAM_VIDEO}" "${OLAY_LOG}"; then
  renk_ok "altyazılı video hazır"
else
  renk_hata "video düzenlenemedi, ham kayıt korunuyor"
  echo "yeniden denemek için:"
  echo "  python3 scripts/edit_video.py ${HAM_VIDEO} ${OLAY_LOG}"
fi
