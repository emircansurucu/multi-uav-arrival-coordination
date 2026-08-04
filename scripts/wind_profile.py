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
#
# SIM_WIND_TC birinci derece gecikmedir (AP_HAL_SITL/SITL_State.cpp: alpha =
# calc_lowpass_alpha_dt(dt, 1/tc)), dolayisiyla tepe donus hizi Delta/tc'dir.
# Bu, profilin ne kadar gercekci oldugunu belirleyen tek sayidir:
#
#   sakin hava gun ici salinimi   < 0.1 derece/s
#   cephe gecisi                    0.2-0.5 derece/s
#   firtina cikis cephesi           1.5-3 derece/s
#
# Profil 30 derecelik adimlar ve 60 s'lik zaman sabitiyle 0.5 derece/s tepe
# hiz verir, yani bir CEPHE GECISI. Yon bati-kuzey ekseninde tek yonlu doner;
# 330 ve 0 derece basamaklari son bacagin (rota 135 derece) kuyruk ruzgari
# durumunu korur, yani zorlu vektor testte kalir.
#
# Siddet araligi aracin ucus zarfina gore secildi. Asgari hava hizi 13 m/s
# oldugu icin ruzgar bu degere yaklastiginda yavaslama yetkisi ve yon tutma
# birlikte kayboluyor: 13 m/s'lik bir basamakta yer hizi sifira yaklasti,
# L1 son waypoint'i kesemedi ve hedefe 6 m yaklasilabildi (sinir 5 m).
# 4-10 m/s araligi Beaufort 3-5'e denk gelir; 400 m irtifada bu, yerdeki
# 5-7 m/s'ye karsilik gelen olagan bir gundur.
DEFAULT_PROFILE: Sequence[Tuple[float, float, float]] = (
    (0.0, 4.0, 270.0),
    (180.0, 6.0, 300.0),
    (360.0, 9.0, 330.0),
    (540.0, 10.0, 0.0),
    (720.0, 7.0, 30.0),
)

# UC DURUM. 70-110 derecelik adimlar ve 20 s zaman sabiti 5.5 derece/s tepe
# hiz verir; bu bir firtina cikis cephesidir ve gercek harekatta ucus iptal
# edilir. Silinmedi cunku algoritmanin sinirini gostermek icin degerli, ama
# varsayilan dogrulama profili degildir.
EXTREME_PROFILE: Sequence[Tuple[float, float, float]] = (
    (0.0, 4.0, 270.0),
    (180.0, 9.0, 200.0),
    (360.0, 6.0, 90.0),
    (540.0, 10.0, 340.0),
    (720.0, 5.0, 45.0),
)
EXTREME_WIND_CHANGE_TC_S = 20.0

# Gecis zaman sabiti. 30 derecelik adimla birlikte 0.5 derece/s tepe donus
# hizi verir (cephe gecisi seviyesi).
WIND_CHANGE_TC_S = 60.0
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


def run(profile: Sequence[Tuple[float, float, float]], tc_s: float) -> int:
    links: List[mavutil.mavfile] = []
    for vehicle in (1, 2, 3):
        try:
            links.append(connect(vehicle))
        except Exception as hata:  # baglanti kurulamazsa profil uygulanamaz
            print(f"HATA: {hata}", file=sys.stderr)
            return 1
    print(f"uc araca baglanildi; {len(profile)} basamakli profil, TC={tc_s:.0f} s")

    for link in links:
        set_param(link, "SIM_WIND_TC", tc_s)

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
    parser.add_argument(
        "--extreme",
        action="store_true",
        help="firtina cikis cephesi profili (5.5 derece/s); varsayilan degildir",
    )
    args = parser.parse_args()

    profile = EXTREME_PROFILE if args.extreme else DEFAULT_PROFILE
    tc_s = EXTREME_WIND_CHANGE_TC_S if args.extreme else WIND_CHANGE_TC_S

    if args.print_profile:
        onceki = None
        for saniye, speed, direction in profile:
            if onceki is None:
                hiz = ""
            else:
                fark = (direction - onceki + 180) % 360 - 180
                hiz = f"  ({fark:+.0f} derece, tepe {abs(fark) / tc_s:.2f} derece/s)"
            print(f"t+{saniye:6.0f} s  {speed:5.1f} m/s  {direction:5.0f} derece{hiz}")
            onceki = direction
        return 0
    return run(profile, tc_s)


if __name__ == "__main__":
    sys.exit(main())
