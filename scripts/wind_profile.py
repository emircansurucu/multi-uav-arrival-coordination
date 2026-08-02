#!/usr/bin/env python3
"""Ucus sirasinda ruzgarin siddetini ve yonunu degistirir.

Vaka dokumani madde 7 "degisken siddet ve yonde ruzgar" istiyor. Parametre
dosyasiyla verilen ruzgar sabit kalir; turbulans (SIM_WIND_TURB) yalnizca
ortalamanin etrafinda salinim uretir, ortalamayi degistirmez. Gercekten
degisken bir profil icin SIM_WIND_SPD ve SIM_WIND_DIR ucus sirasinda
guncellenmelidir.

SITL yeni degere aninda atlamaz: SIM_WIND_TC zaman sabitiyle yumusak gecis
yapar, dolayisiyla profil basamakli verilse de ruzgar surekli degisir.

SERIAL0'i (5760 + 10*instance) arac agent'i kullaniyor ve SITL'in tcp portu
tek istemci kabul ediyor; bu yuzden start_sitl.sh ikinci bir port aciyor
(--serial1=tcp:2 -> 5762 + 10*instance) ve bu betik oraya baglanir.
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import List, Sequence, Tuple

from pymavlink import mavutil

# (gorev basindan itibaren saniye, hiz m/s, ruzgarin GELDIGI yon derece)
# Profil hem siddeti hem yonu degistirir; yon 270 -> 200 -> 90 -> 340 -> 45
# ile dort kadrani da dolasir, boylece her rota bacagi farkli bilesenler gorur.
#
# Siddet araligi aracin ucus zarfina gore secildi. Asgari hava hizi 13 m/s
# oldugu icin ruzgar bu degere yaklastiginda yavaslama yetkisi ve yon tutma
# birlikte kayboluyor: 13 m/s'lik bir basamakta yer hizi sifira yaklasti,
# L1 son waypoint'i kesemedi ve hedefe 6 m yaklasilabildi (sinir 5 m).
# 4-10 m/s araligi hem 2.5 kat siddet degisimi verir hem de araca her
# durumda ilerleme birakir. Doküman ruzgar siddeti icin bir deger belirtmiyor.
DEFAULT_PROFILE: Sequence[Tuple[float, float, float]] = (
    (0.0, 4.0, 270.0),
    (180.0, 9.0, 200.0),
    (360.0, 6.0, 90.0),
    (540.0, 10.0, 340.0),
    (720.0, 5.0, 45.0),
)
# Gecis zaman sabiti. Varsayilan 5 s ani sayilabilecek kadar kisa; daha
# uzun bir sabit ruzgarin gercekci sekilde donmesini saglar.
WIND_CHANGE_TC_S = 20.0
SERIAL1_PORT_BASE = 5762
PORT_STRIDE = 10
CONNECT_TIMEOUT_S = 60


def connect(vehicle: int) -> mavutil.mavfile:
    port = SERIAL1_PORT_BASE + PORT_STRIDE * (vehicle - 1)
    link = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}")
    link.wait_heartbeat(timeout=CONNECT_TIMEOUT_S)
    if link.target_system == 0:
        raise ConnectionError(f"HA-{vehicle}: {port} uzerinden heartbeat alinamadi")
    return link


def set_param(link: mavutil.mavfile, name: str, value: float) -> None:
    link.mav.param_set_send(
        link.target_system,
        link.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def apply_step(links: Sequence[mavutil.mavfile], speed: float, direction: float) -> None:
    for link in links:
        set_param(link, "SIM_WIND_SPD", speed)
        set_param(link, "SIM_WIND_DIR", direction)


def run(profile: Sequence[Tuple[float, float, float]]) -> int:
    links: List[mavutil.mavfile] = []
    for vehicle in (1, 2, 3):
        try:
            links.append(connect(vehicle))
        except Exception as hata:  # baglanti kurulamazsa profil uygulanamaz
            print(f"HATA: {hata}", file=sys.stderr)
            return 1
    print(f"uc araca baglanildi; {len(profile)} basamakli profil uygulanacak")

    for link in links:
        set_param(link, "SIM_WIND_TC", WIND_CHANGE_TC_S)

    baslangic = time.monotonic()
    for saniye, speed, direction in profile:
        bekleme = saniye - (time.monotonic() - baslangic)
        if bekleme > 0:
            time.sleep(bekleme)
        apply_step(links, speed, direction)
        print(f"t+{saniye:6.0f} s | ruzgar {speed:5.1f} m/s | {direction:5.0f} dereceden")
    print("profil tamamlandi")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--print-profile",
        action="store_true",
        help="profili uygulamadan yazdirir (rapor icin)",
    )
    args = parser.parse_args()

    if args.print_profile:
        for saniye, speed, direction in DEFAULT_PROFILE:
            print(f"t+{saniye:6.0f} s  {speed:5.1f} m/s  {direction:5.0f} derece")
        return 0
    return run(DEFAULT_PROFILE)


if __name__ == "__main__":
    sys.exit(main())
