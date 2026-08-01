#!/usr/bin/env bash
# Tum SITL, XRCE agent ve agent node sureclerini durdurur ve portlarin
# serbest kalmasini bekler.
#
# Desenlerde koseli parantez kullanilir; aksi halde pgrep bu betigin kendi
# komut satirini de eslestirip kendini oldurur.
#
# Port beklemesi zorunlu: SITL hemen yeniden baslatilirsa TCP 5760 ve UDP
# 2019 hala tutulu oldugu icin arduplane bind edemeden sessizce cikiyor.
set -uo pipefail

PORT_WAIT_ATTEMPTS=30
PORT_WAIT_INTERVAL_S=1

PATTERNS=(
  "oasy_uav_agent/agent_nod[e]"
  "three_vehicles.launc[h].py"
  "sim_vehic[l]e.py"
  "bin/ardupla[n]e"
  "xter[m] -iconic"
  "lib/micro_ros_agent/micro_ros_age[n]t"
  "ros2 run micro_ros_age[n]t"
)

for pattern in "${PATTERNS[@]}"; do
  pids="$(pgrep -f "${pattern}" || true)"
  if [[ -n "${pids}" ]]; then
    echo "durduruluyor: ${pattern} (${pids//$'\n'/ })"
    # shellcheck disable=SC2086
    kill ${pids} 2>/dev/null || true
  fi
done

ports_busy() {
  ss -ltn 2>/dev/null | grep -qE ':(5760|5770|5780)\b' && return 0
  ss -lun 2>/dev/null | grep -qE ':(2019|2020|2021)\b' && return 0
  return 1
}

for _ in $(seq 1 "${PORT_WAIT_ATTEMPTS}"); do
  ports_busy || break
  sleep "${PORT_WAIT_INTERVAL_S}"
done

if ports_busy; then
  echo "UYARI: portlar hala tutulu, kalan surecler zorla kapatiliyor" >&2
  for pattern in "${PATTERNS[@]}"; do
    pids="$(pgrep -f "${pattern}" || true)"
    # shellcheck disable=SC2086
    [[ -n "${pids}" ]] && kill -9 ${pids} 2>/dev/null || true
  done
  sleep 2
fi

echo "durduruldu"
