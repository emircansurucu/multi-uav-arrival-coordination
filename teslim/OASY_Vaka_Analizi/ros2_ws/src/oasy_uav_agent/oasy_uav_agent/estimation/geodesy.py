"""jeodezik mesafe ve kabul çemberi hesaplarını yapar"""
from __future__ import annotations

import math
from typing import NamedTuple, Optional, Sequence, Tuple

from geographiclib.geodesic import Geodesic

_GEOD = Geodesic.WGS84  # wgs84 jeodezik hesaplayıcısı
_EARTH_RADIUS_M = 6378137.0  # yerel dönüşümde kullanılan dünya yarıçapı


class LatLon(NamedTuple):
    lat: float
    lon: float


def geodesic_distance_m(a: LatLon, b: LatLon) -> float:
    # wgs84 elipsoidi üzerindeki yatay mesafeyi hesaplar
    return _GEOD.Inverse(a.lat, a.lon, b.lat, b.lon)["s12"]


def initial_bearing_deg(a: LatLon, b: LatLon) -> float:
    # a noktasından b noktasına kerterizi hesaplar
    return _GEOD.Inverse(a.lat, a.lon, b.lat, b.lon)["azi1"] % 360.0


def to_local_xy(point: LatLon, origin: LatLon) -> Tuple[float, float]:
    # konumu merkez noktasına göre doğu ve kuzey metrelerine çevirir
    lat_rad = math.radians(origin.lat)
    east = math.radians(point.lon - origin.lon) * _EARTH_RADIUS_M * math.cos(lat_rad)
    north = math.radians(point.lat - origin.lat) * _EARTH_RADIUS_M
    return east, north


def from_local_xy(east_m: float, north_m: float, origin: LatLon) -> LatLon:
    # yerel metre değerlerini enlem ve boylama çevirir
    lat_rad = math.radians(origin.lat)
    latitude = origin.lat + math.degrees(north_m / _EARTH_RADIUS_M)
    longitude = origin.lon + math.degrees(east_m / (_EARTH_RADIUS_M * math.cos(lat_rad)))
    return LatLon(latitude, longitude)


def cross_track_distance_m(position: LatLon, leg_start: LatLon, leg_end: LatLon) -> float:
    # konumun rota bacağına en kısa uzaklığını hesaplar
    origin = leg_start
    leg = to_local_xy(leg_end, origin)
    point = to_local_xy(position, origin)

    leg_squared = leg[0] ** 2 + leg[1] ** 2
    if leg_squared == 0.0:
        return math.hypot(*point)

    fraction = (point[0] * leg[0] + point[1] * leg[1]) / leg_squared
    fraction = max(0.0, min(1.0, fraction))
    closest = (fraction * leg[0], fraction * leg[1])
    return math.hypot(point[0] - closest[0], point[1] - closest[1])


def circle_entry_fraction(
    start: Tuple[float, float],
    end: Tuple[float, float],
    radius_m: float,
) -> Optional[float]:
    # doğru parçasının çembere ilk giriş oranını hesaplar
    x1, y1 = start
    dx = end[0] - x1
    dy = end[1] - y1

    a = dx * dx + dy * dy
    if a == 0.0:
        return None

    c = x1 * x1 + y1 * y1 - radius_m * radius_m
    if c <= 0.0:
        # başlangıç noktası çemberin içinde
        return 0.0

    b = 2.0 * (x1 * dx + y1 * dy)
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return None

    root = math.sqrt(discriminant)
    entry = (-b - root) / (2.0 * a)
    return entry if 0.0 <= entry <= 1.0 else None


def _refine_circle_entry(
    start_xy: Tuple[float, float],
    end_xy: Tuple[float, float],
    center: LatLon,
    radius_m: float,
    fraction: float,
) -> LatLon:
    # düzlem çözümünü jeodezik mesafeye göre düzeltir
    def point(f: float) -> LatLon:
        # bacak üzerindeki oranı küresel konuma çevirir
        return from_local_xy(
            start_xy[0] + f * (end_xy[0] - start_xy[0]),
            start_xy[1] + f * (end_xy[1] - start_xy[1]),
            center,
        )

    # çözümü bacağın yakın çevresinde arar
    lo = max(0.0, fraction - 0.02)
    hi = min(1.0, fraction + 0.02)
    if geodesic_distance_m(point(lo), center) < radius_m:
        lo = 0.0
    if (
        geodesic_distance_m(point(lo), center) < radius_m
        or geodesic_distance_m(point(hi), center) > radius_m
    ):
        # uygun aralık yoksa düzlem çözümünü korur
        return point(fraction)
    for _ in range(40):
        midpoint = 0.5 * (lo + hi)
        if geodesic_distance_m(point(midpoint), center) > radius_m:
            lo = midpoint
        else:
            hi = midpoint
    return point(0.5 * (lo + hi))


def last_circle_entry_on_route(
    home: LatLon,
    route: Sequence[LatLon],
    center: LatLon,
    radius_m: float,
) -> Optional[Tuple[LatLon, int]]:
    # rotanın bir çembere son dışarıdan girişini bulur
    if not route or radius_m <= 0.0:
        return None

    result: Optional[Tuple[LatLon, int]] = None
    points = (home, *route)
    for active_index, (start, end) in enumerate(zip(points, points[1:])):
        start_xy = to_local_xy(start, center)
        # içeride başlayan bacağı atlar
        if math.hypot(*start_xy) <= radius_m:
            continue
        end_xy = to_local_xy(end, center)
        fraction = circle_entry_fraction(start_xy, end_xy, radius_m)
        if fraction is None:
            continue
        result = (
            _refine_circle_entry(start_xy, end_xy, center, radius_m, fraction),
            active_index,
        )
    return result
