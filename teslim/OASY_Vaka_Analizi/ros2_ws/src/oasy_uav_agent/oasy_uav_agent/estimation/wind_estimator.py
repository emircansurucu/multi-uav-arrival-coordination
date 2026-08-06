"""rüzgârı telemetriden kestirir ve rota süresine uygular"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from .geodesy import LatLon, geodesic_distance_m, to_local_xy


WIND_FILTER_TIME_CONSTANT_S = 2.5  # rüzgâr filtresinin zaman sabiti
WIND_SETTLE_AFTER_S = 7.5  # rüzgâr kestiriminin oturma süresi


@dataclass(frozen=True)
class WindEstimate:
    # kestirilen doğu ve kuzey rüzgâr bileşenlerini tutar

    east_mps: float
    north_mps: float

    @property
    def speed_mps(self) -> float:
        # rüzgar vektörünün büyüklüğünü döner
        return math.hypot(self.east_mps, self.north_mps)

    @property
    def from_direction_deg(self) -> float:
        # rüzgarın geldiği meteorolojik yönü döndürür
        return (math.degrees(math.atan2(-self.east_mps, -self.north_mps))) % 360.0


def wind_from_speed_direction(speed_mps: float, from_direction_deg: float) -> WindEstimate:
    # rüzgar hızı ve geliş yönünü enu vektörüne çevirir
    radians = math.radians(from_direction_deg)
    return WindEstimate(
        east_mps=-speed_mps * math.sin(radians),
        north_mps=-speed_mps * math.cos(radians),
    )


def body_to_enu(
    vector_flu: Tuple[float, float, float],
    orientation_xyzw: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    # flu gövde vektörünü yönelimle enu sistemine çevirir
    x, y, z, w = orientation_xyzw
    vx, vy, vz = vector_flu

    # vektörü kuaterniyonla döndürür
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
    # yer ve hava hızı farkından yatay rüzgârı kestirir
    if math.sqrt(sum(component ** 2 for component in airspeed_body_flu)) <= 0.0:
        return None

    east, north, _up = body_to_enu(airspeed_body_flu, orientation_xyzw)
    return WindEstimate(
        east_mps=ground_velocity_en[0] - east,
        north_mps=ground_velocity_en[1] - north,
    )


class WindFilter:
    # ham rüzgâr örneklerini süzer ve oturma durumunu izler

    def __init__(
        self,
        time_constant_s: float = WIND_FILTER_TIME_CONSTANT_S,
        settle_after_s: float = WIND_SETTLE_AFTER_S,
    ) -> None:
        # filtre sabitlerini ve başlangıç rüzgâr değerlerini hazırlar
        self._time_constant_s = time_constant_s
        self._settle_after_s = settle_after_s
        self._east_mps = 0.0
        self._north_mps = 0.0
        self._elapsed_s = 0.0
        self._started = False

    def update(self, sample: WindEstimate, dt_s: float) -> None:
        # yeni rüzgâr örneğini geçen süreye göre filtreye ekler
        if dt_s <= 0.0:
            return
        if not self._started:
            self._east_mps = sample.east_mps
            self._north_mps = sample.north_mps
            self._started = True
        else:
            # filtre ağırlığını örnek aralığına göre hesaplar
            alpha = dt_s / (self._time_constant_s + dt_s)
            self._east_mps += alpha * (sample.east_mps - self._east_mps)
            self._north_mps += alpha * (sample.north_mps - self._north_mps)
        self._elapsed_s += dt_s

    @property
    def estimate(self) -> Optional[WindEstimate]:
        # filtre başlatıldıysa güncel rüzgâr kestirimini döner
        if not self._started:
            return None
        return WindEstimate(east_mps=self._east_mps, north_mps=self._north_mps)

    @property
    def settled(self) -> bool:
        # filtrenin yeterli süre çalışıp çalışmadığını döner
        return self._started and self._elapsed_s >= self._settle_after_s


def route_duration_with_wind_s(
    home: LatLon,
    route: Sequence[LatLon],
    airspeed_mps: float,
    wind: WindEstimate,
) -> float:
    # rotanın rüzgâr altındaki beklenen uçuş süresini hesaplar
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
    # hız değişim sınırını içeren rota süresini hesaplar
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
            # kalan yolu hedef hızla tamamlar
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

        # rota noktası geçilirse adımın kullanılan bölümünü hesaplar
        used_fraction = remaining_leg_m / max(travelled_m, 1e-9)
        used_dt_s = dt_s * used_fraction
        elapsed_s += used_dt_s
        airspeed_mps += direction * rate_limit_mps2 * used_dt_s
        remaining_leg_m = 0.0

    return elapsed_s


def _ground_speed_along_leg_mps(
    start: LatLon, end: LatLon, airspeed_mps: float, wind: WindEstimate
) -> float:
    # rota bacağı yönündeki yer hızını hesaplar
    east_m, north_m = to_local_xy(end, start)
    length_m = math.hypot(east_m, north_m)
    along = (east_m / length_m, north_m / length_m)

    # yan rüzgârın hava hızından kullandığı payı düşer
    wind_along = wind.east_mps * along[0] + wind.north_mps * along[1]
    wind_cross = -wind.east_mps * along[1] + wind.north_mps * along[0]

    if abs(wind_cross) >= airspeed_mps:
        # rota doğrultusu korunamıyorsa alt sınırı kullanır
        return _MIN_GROUND_SPEED_MPS

    forward = math.sqrt(airspeed_mps ** 2 - wind_cross ** 2) + wind_along
    return max(forward, _MIN_GROUND_SPEED_MPS)


_MIN_GROUND_SPEED_MPS = 1.0  # süre hesabındaki en düşük yer hızı
