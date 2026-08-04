"""Ruzgar vektorunun telemetriden kestirimi ve rota suresine etkisi.

AP_DDS ruzgar tahminini yayinlamaz, ama gereken her seyi yayinlar:
yer hizi vektoru (/ap/twist/filtered, ENU) ve gercek hava hizi vektoru
(/ap/airspeed, govde cercevesinde FLU). Aracin yonelimi geopose'dan
alinip hava hizi vektoru ENU'ya donusturuldugunde ruzgar tam olarak:

    ruzgar = yer_hizi_vektoru - hava_hizi_vektoru

Hava hizinin yalnizca buyuklugu kullanilsaydi burun dogrultusu bilinmedigi
icin crab acisi kadar hata kalirdi; vektor yayinlandigi icin yaklasim
gerekmiyor.

Bu modul, kalkis oncesi nominal ucus suresini duzeltmek icin kullanilir:
ruzgarsiz hesaplanan sure ruzgar altinda ulasilamaz oldugunda, plan bastan
yanlis kurulur ve havadayken duzeltilemeyecek kadar buyuk hata birikir.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from .geodesy import LatLon, geodesic_distance_m, to_local_xy


# AP_DDS konum, yer hizi ve hava hizini 33 ms'de bir yayinlar; ornek basina
# gurultu plani kaydiracak kadar buyuk olabiliyor. 2.5 s'lik zaman sabiti
# gurultuyu bastirirken gercek ruzgar degisimini gec birakmayacak kadar kisa.
WIND_FILTER_TIME_CONSTANT_S = 2.5
# Uc zaman sabiti: filtre son degerin ~%95'ine ulasir. Bu sureden once
# kestirim "oturmus" sayilmaz ve plan hesabinda kullanilmaz.
WIND_SETTLE_AFTER_S = 7.5


@dataclass(frozen=True)
class WindEstimate:
    """Kestirilen ruzgar vektoru (ENU, m/s)."""

    east_mps: float
    north_mps: float

    @property
    def speed_mps(self) -> float:
        return math.hypot(self.east_mps, self.north_mps)

    @property
    def from_direction_deg(self) -> float:
        """Ruzgarin GELDIGI yon (meteorolojik, kuzeyden saat yonunde)."""
        return (math.degrees(math.atan2(-self.east_mps, -self.north_mps))) % 360.0


def wind_from_speed_direction(speed_mps: float, from_direction_deg: float) -> WindEstimate:
    """Meteorolojik (hiz, gelis yonu) ciftinden ENU ruzgar vektoru."""
    radians = math.radians(from_direction_deg)
    return WindEstimate(
        east_mps=-speed_mps * math.sin(radians),
        north_mps=-speed_mps * math.cos(radians),
    )


def body_to_enu(
    vector_flu: Tuple[float, float, float],
    orientation_xyzw: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """FLU govde vektorunu tam yonelimle ENU'ya cevirir.

    Yalnizca yaw ile dondurmek yeterli degil: AP_DDS'in yayinladigi hava
    hizi vektoru EKF tarafindan tam yonelimle (roll+pitch+yaw) govde
    cercevesine dondurulmus durumda (NavEKF3_core::getAirSpdVec). Sadece
    yaw'i ters cevirmek yalnizca duz ve seviye ucusta dogru sonuc verir;
    tirmanista pitch, donuslerde roll kadar hata birakir. Olculen etki
    kucuk degil: tirmanista 8 m/s'lik ruzgar 6.3 m/s olarak kestiriliyordu.
    """
    x, y, z, w = orientation_xyzw
    vx, vy, vz = vector_flu

    # v' = v + 2w(q x v) + 2(q x (q x v)) -- kuaterniyonla dondurme.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def estimate_wind(
    ground_velocity_en: Tuple[float, float],
    airspeed_body_flu: Tuple[float, float, float],
    orientation_xyzw: Tuple[float, float, float, float],
) -> Optional[WindEstimate]:
    """Yer hizi ve hava hizi vektorlerinin farkindan ruzgari kestirir.

    Ruzgar yatay bir buyukluk olarak modellenir; hava hizi vektorunun ENU
    karsiliginin yalnizca yatay bilesenleri kullanilir. Dikey bilesen
    (tirmanis/alcalma) rota suresini etkilemez.
    """
    if math.sqrt(sum(component ** 2 for component in airspeed_body_flu)) <= 0.0:
        return None

    east, north, _up = body_to_enu(airspeed_body_flu, orientation_xyzw)
    return WindEstimate(
        east_mps=ground_velocity_en[0] - east,
        north_mps=ground_velocity_en[1] - north,
    )


class WindFilter:
    """Ham ruzgar orneklerini suzer ve kestirimin oturdugunu bildirir.

    Filtre ENU bilesenleri uzerinde calisir, (hiz, yon) cifti uzerinde
    degil: kuzey ruzgarinda yon 0 ile 360 arasinda salinir ve derecelerin
    ortalamasi anlamsiz bir deger uretir. Vektor ortalamasinda bu sorun yok.
    """

    def __init__(
        self,
        time_constant_s: float = WIND_FILTER_TIME_CONSTANT_S,
        settle_after_s: float = WIND_SETTLE_AFTER_S,
    ) -> None:
        self._time_constant_s = time_constant_s
        self._settle_after_s = settle_after_s
        self._east_mps = 0.0
        self._north_mps = 0.0
        self._elapsed_s = 0.0
        self._started = False

    def update(self, sample: WindEstimate, dt_s: float) -> None:
        if dt_s <= 0.0:
            return
        if not self._started:
            self._east_mps = sample.east_mps
            self._north_mps = sample.north_mps
            self._started = True
        else:
            # Agirlik dt'den turetilir; sabit bir katsayi, ornek araligi
            # degistiginde filtrenin zaman sabitini de degistirirdi.
            alpha = dt_s / (self._time_constant_s + dt_s)
            self._east_mps += alpha * (sample.east_mps - self._east_mps)
            self._north_mps += alpha * (sample.north_mps - self._north_mps)
        self._elapsed_s += dt_s

    @property
    def estimate(self) -> Optional[WindEstimate]:
        if not self._started:
            return None
        return WindEstimate(east_mps=self._east_mps, north_mps=self._north_mps)

    @property
    def settled(self) -> bool:
        return self._started and self._elapsed_s >= self._settle_after_s


def route_duration_with_wind_s(
    home: LatLon,
    route: Sequence[LatLon],
    airspeed_mps: float,
    wind: WindEstimate,
) -> float:
    """Rotanin ruzgar altinda beklenen ucus suresi.

    Her bacak icin yer hizi, hava hizinin bacak dogrultusundaki bileseni ile
    ruzgarin ayni dogrultudaki bileseninden bulunur. Yan ruzgarda ucak
    ruzgara karsi kirar (crab), bu yuzden ileri bilesen azalir; hesap bunu
    dikkate alir.
    """
    if airspeed_mps <= 0.0:
        raise ValueError("hava hizi pozitif olmali")

    total_s = 0.0
    points = [home, *route]
    for start, end in zip(points, points[1:]):
        leg_length_m = geodesic_distance_m(start, end)
        if leg_length_m <= 0.0:
            continue
        total_s += leg_length_m / _ground_speed_along_leg_mps(
            start, end, airspeed_mps, wind
        )
    return total_s


def route_duration_with_airspeed_ramp_s(
    home: LatLon,
    route: Sequence[LatLon],
    initial_airspeed_mps: float,
    target_airspeed_mps: float,
    rate_limit_mps2: float,
    wind: WindEstimate,
    integration_step_s: float = 0.25,
) -> float:
    """Hiz sinirina anlik degil rate-limit ile ulasilan rota suresi.

    E/L sinirlari min/max hava hizina bir tick'te gecilebildigini varsayarsa
    kalan kontrol yetkisini iyimser gosterir. Yalnizca kisa hiz rampasi
    sayisal olarak entegre edilir; hedef hiza ulasildiktan sonraki rota
    analitik bacak modeliyle tamamlanir. Boylece maliyet rota uzunluguyla
    degil, en fazla hiz gecisinin suresiyle sinirlidir.
    """
    if initial_airspeed_mps <= 0.0 or target_airspeed_mps <= 0.0:
        raise ValueError("hava hizlari pozitif olmali")
    if rate_limit_mps2 <= 0.0:
        raise ValueError("hiz degisim siniri pozitif olmali")
    if integration_step_s <= 0.0:
        raise ValueError("entegrasyon adimi pozitif olmali")
    if not route:
        return 0.0
    if math.isclose(initial_airspeed_mps, target_airspeed_mps, abs_tol=1e-9):
        return route_duration_with_wind_s(home, route, target_airspeed_mps, wind)

    points = (home, *route)
    leg_index = 0
    remaining_leg_m = geodesic_distance_m(points[0], points[1])
    airspeed_mps = initial_airspeed_mps
    elapsed_s = 0.0
    direction = 1.0 if target_airspeed_mps > initial_airspeed_mps else -1.0

    while leg_index < len(points) - 1:
        if remaining_leg_m <= 1e-6:
            leg_index += 1
            if leg_index >= len(points) - 1:
                return elapsed_s
            remaining_leg_m = geodesic_distance_m(
                points[leg_index], points[leg_index + 1]
            )
            continue

        speed_gap_mps = abs(target_airspeed_mps - airspeed_mps)
        if speed_gap_mps <= 1e-9:
            # Rampadan kalan bacak parcasi ve sonraki tam bacaklar.
            elapsed_s += remaining_leg_m / _ground_speed_along_leg_mps(
                points[leg_index], points[leg_index + 1], target_airspeed_mps, wind
            )
            for start, end in zip(points[leg_index + 1:], points[leg_index + 2:]):
                length_m = geodesic_distance_m(start, end)
                if length_m > 0.0:
                    elapsed_s += length_m / _ground_speed_along_leg_mps(
                        start, end, target_airspeed_mps, wind
                    )
            return elapsed_s

        dt_s = min(integration_step_s, speed_gap_mps / rate_limit_mps2)
        next_airspeed_mps = airspeed_mps + direction * rate_limit_mps2 * dt_s
        ground_before_mps = _ground_speed_along_leg_mps(
            points[leg_index], points[leg_index + 1], airspeed_mps, wind
        )
        ground_after_mps = _ground_speed_along_leg_mps(
            points[leg_index], points[leg_index + 1], next_airspeed_mps, wind
        )
        travelled_m = 0.5 * (ground_before_mps + ground_after_mps) * dt_s

        if travelled_m < remaining_leg_m:
            remaining_leg_m -= travelled_m
            elapsed_s += dt_s
            airspeed_mps = next_airspeed_mps
            continue

        # Rampa bir waypoint'i geciyorsa adimin yalniz o bacaga dusen
        # kismini kullan; sonraki bacakta yeni dogrultuyla tekrar hesapla.
        used_fraction = remaining_leg_m / max(travelled_m, 1e-9)
        used_dt_s = dt_s * used_fraction
        elapsed_s += used_dt_s
        airspeed_mps += direction * rate_limit_mps2 * used_dt_s
        remaining_leg_m = 0.0

    return elapsed_s


def _ground_speed_along_leg_mps(
    start: LatLon, end: LatLon, airspeed_mps: float, wind: WindEstimate
) -> float:
    """Bacak dogrultusunda elde edilebilecek yer hizi."""
    east_m, north_m = to_local_xy(end, start)
    length_m = math.hypot(east_m, north_m)
    along = (east_m / length_m, north_m / length_m)

    # Ruzgarin bacaga dik bileseni crab ile dengelenir; bu, ileri yonde
    # kullanilabilir hava hizini azaltir.
    wind_along = wind.east_mps * along[0] + wind.north_mps * along[1]
    wind_cross = -wind.east_mps * along[1] + wind.north_mps * along[0]

    if abs(wind_cross) >= airspeed_mps:
        # Yan ruzgar hava hizini asiyorsa bacak dogrultusu tutturulamaz.
        return _MIN_GROUND_SPEED_MPS

    forward = math.sqrt(airspeed_mps ** 2 - wind_cross ** 2) + wind_along
    return max(forward, _MIN_GROUND_SPEED_MPS)


# Ruzgar hava hizini astiginda sure sonsuza gitmesin diye alt sinir.
_MIN_GROUND_SPEED_MPS = 1.0
