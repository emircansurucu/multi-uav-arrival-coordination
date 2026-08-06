#!/usr/bin/env python3
"""uçuş sırasında rüzgâr hızını ve yönünü değiştirir"""
from __future__ import annotations

import argparse
import sys
import time
from typing import List, Sequence, Tuple

from pymavlink import mavutil

DEFAULT_PROFILE: Sequence[Tuple[float, float, float]] = (
    (0.0, 4.0, 270.0),
    (180.0, 6.0, 300.0),
    (360.0, 9.0, 330.0),
    (540.0, 10.0, 0.0),
    (720.0, 7.0, 30.0),
)  # normal değişken rüzgâr basamakları

EXTREME_PROFILE: Sequence[Tuple[float, float, float]] = (
    (0.0, 4.0, 270.0),
    (180.0, 9.0, 200.0),
    (360.0, 6.0, 90.0),
    (540.0, 10.0, 340.0),
    (720.0, 5.0, 45.0),
)  # uç durum rüzgâr basamakları
EXTREME_WIND_CHANGE_TC_S = 20.0  # uç durum geçiş zaman sabiti

WIND_CHANGE_TC_S = 60.0  # normal profil geçiş zaman sabiti
SERIAL1_PORT_BASE = 5762  # ilk aracın ikinci seri bağlantı portu
PORT_STRIDE = 10  # araç portları arasındaki fark
CONNECT_TIMEOUT_S = 60  # mavlink bağlantı zaman aşımı


def connect(vehicle: int) -> mavutil.mavfile:
    # seçilen aracın sitl mavlink bağlantısını açar
    port = SERIAL1_PORT_BASE + PORT_STRIDE * (vehicle - 1)
    link = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}")
    link.wait_heartbeat(timeout=CONNECT_TIMEOUT_S)
    if link.target_system == 0:
        raise ConnectionError(f"HA-{vehicle}: {port} uzerinden heartbeat alinamadi")
    return link


def set_param(link: mavutil.mavfile, name: str, value: float) -> None:
    # araçtaki bir simülasyon parametresini mavlink ile ayarlar
    link.mav.param_set_send(
        link.target_system,
        link.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def apply_step(links: Sequence[mavutil.mavfile], speed: float, direction: float) -> None:
    # aynı rüzgâr basamağını bütün araçlara uygular
    for link in links:
        set_param(link, "SIM_WIND_SPD", speed)
        set_param(link, "SIM_WIND_DIR", direction)


def run(profile: Sequence[Tuple[float, float, float]], tc_s: float) -> int:
    # seçilen rüzgâr profilini zaman sırasıyla üç araçta çalıştırır
    links: List[mavutil.mavfile] = []
    for vehicle in (1, 2, 3):
        try:
            links.append(connect(vehicle))
        except Exception as hata:  # bağlantı yoksa profili durdurur
            print(f"HATA: {hata}", file=sys.stderr)
            return 1
    print(f"uc araca baglanildi; {len(profile)} basamakli profil, TC={tc_s:.0f} s")

    for link in links:
        set_param(link, "SIM_WIND_TC", tc_s)

    start = time.monotonic()
    for second, speed, direction in profile:
        wait_s = second - (time.monotonic() - start)
        if wait_s > 0:
            time.sleep(wait_s)
        apply_step(links, speed, direction)
        print(f"t+{second:6.0f} s | rüzgar {speed:5.1f} m/s | {direction:5.0f} dereceden")
    print("profil tamamlandı")
    return 0


def main() -> int:
    # rüzgar profili seçimini okuyup gösterir veya uygular
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--print-profile",
        action="store_true",
        help="profili uygulamadan yazdirir (rapor icin)",
    )
    parser.add_argument(
        "--extreme",
        action="store_true",
        help="firtina cikis cephesi profili (5.5 derece/s); varsayilan degildir",
    )
    args = parser.parse_args()

    profile = EXTREME_PROFILE if args.extreme else DEFAULT_PROFILE
    tc_s = EXTREME_WIND_CHANGE_TC_S if args.extreme else WIND_CHANGE_TC_S

    if args.print_profile:
        previous = None
        for second, speed, direction in profile:
            if previous is None:
                rate_text = ""
            else:
                delta = (direction - previous + 180) % 360 - 180
                rate_text = f"  ({delta:+.0f} derece, tepe {abs(delta) / tc_s:.2f} derece/s)"
            print(f"t+{second:6.0f} s  {speed:5.1f} m/s  {direction:5.0f} derece{rate_text}")
            previous = direction
        return 0
    return run(profile, tc_s)


if __name__ == "__main__":
    sys.exit(main())
