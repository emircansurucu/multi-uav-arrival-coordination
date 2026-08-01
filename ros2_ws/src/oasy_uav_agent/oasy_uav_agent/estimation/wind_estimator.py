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


@dataclass(frozen=True)
class WindEstimate:
    """Kestirilen ruzgar vektoru (ENU, m/s) ve guvenilirligi."""

    east_mps: float
    north_mps: float
    sample_count: int

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
        sample_count=1,
    )


def body_to_enu(forward_mps: float, left_mps: float, yaw_rad: float) -> Tuple[float, float]:
    """FLU govde vektorunu ENU duzlemine cevirir.

    ROS ENU'da yaw dogu ekseninden saat yonunun tersine olculur; bu,
    SITL kalkis yonu 11 derece (NED) iken olculen 78.7 derece kuaterniyon
    yaw'i ile dogrulanmistir.
    """
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    east = cos_yaw * forward_mps - sin_yaw * left_mps
    north = sin_yaw * forward_mps + cos_yaw * left_mps
    return east, north


def estimate_wind(
    ground_velocity_en: Tuple[float, float],
    airspeed_body_fl: Tuple[float, float],
    yaw_rad: float,
) -> Optional[WindEstimate]:
    """Yer hizi ve hava hizi vektorlerinin farkindan ruzgari kestirir."""
    if math.hypot(*airspeed_body_fl) <= 0.0:
        return None

    airspeed_en = body_to_enu(airspeed_body_fl[0], airspeed_body_fl[1], yaw_rad)
    return WindEstimate(
        east_mps=ground_velocity_en[0] - airspeed_en[0],
        north_mps=ground_velocity_en[1] - airspeed_en[1],
        sample_count=1,
    )


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
