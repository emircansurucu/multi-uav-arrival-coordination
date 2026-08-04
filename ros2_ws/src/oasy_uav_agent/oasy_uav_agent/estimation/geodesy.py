"""Jeodezik mesafe hesabi ve hedef kabul cemberi kesisim tespiti."""
from __future__ import annotations

import math
from typing import NamedTuple, Optional, Sequence, Tuple

from geographiclib.geodesic import Geodesic

_GEOD = Geodesic.WGS84
_EARTH_RADIUS_M = 6378137.0


class LatLon(NamedTuple):
    lat: float
    lon: float


def geodesic_distance_m(a: LatLon, b: LatLon) -> float:
    """WGS84 elipsoidi uzerinde yatay mesafe."""
    return _GEOD.Inverse(a.lat, a.lon, b.lat, b.lon)["s12"]


def initial_bearing_deg(a: LatLon, b: LatLon) -> float:
    """a noktasindan b'ye kerteriz, 0-360 araliginda."""
    return _GEOD.Inverse(a.lat, a.lon, b.lat, b.lon)["azi1"] % 360.0


def to_local_xy(point: LatLon, origin: LatLon) -> Tuple[float, float]:
    """Origin merkezli duzlemsel metre koordinati (dogu, kuzey).

    Kabul cemberi tespiti yalnizca hedefin birkac yuz metre yakininda
    calistigi icin esdikdortgen yaklasimi milimetre altinda hata birakir.
    """
    lat_rad = math.radians(origin.lat)
    east = math.radians(point.lon - origin.lon) * _EARTH_RADIUS_M * math.cos(lat_rad)
    north = math.radians(point.lat - origin.lat) * _EARTH_RADIUS_M
    return east, north


def from_local_xy(east_m: float, north_m: float, origin: LatLon) -> LatLon:
    """to_local_xy'nin tersi: origin merkezli metre ofsetinden koordinat."""
    lat_rad = math.radians(origin.lat)
    latitude = origin.lat + math.degrees(north_m / _EARTH_RADIUS_M)
    longitude = origin.lon + math.degrees(east_m / (_EARTH_RADIUS_M * math.cos(lat_rad)))
    return LatLon(latitude, longitude)


def cross_track_distance_m(position: LatLon, leg_start: LatLon, leg_end: LatLon) -> float:
    """Konumun leg_start-leg_end dogru parcasina en kisa uzakligi.

    Dokumandaki 500 m rota sapmasi siniri bu olcuye gore denetlenir.
    Bacak uclarinin disinda kalindiginda uc noktaya olan mesafe kullanilir.
    """
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
    """start->end dogru parcasinin, orijindeki cembere ilk giris orani.

    Telemetri ornekleri arasinda kalan bir gecis kacirilmasin diye mesafe
    interpolasyonu yerine gercek kesisim cozulur: iki ornek de cemberin
    disindayken segment cemberi kesip cikmis olabilir.
    Donen deger 0-1 arasi oran, kesisim yoksa None.
    """
    x1, y1 = start
    dx = end[0] - x1
    dy = end[1] - y1

    a = dx * dx + dy * dy
    if a == 0.0:
        return None

    c = x1 * x1 + y1 * y1 - radius_m * radius_m
    if c <= 0.0:
        # Baslangic noktasi zaten cemberin icinde.
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
    """Duzlem izdusumu cozumunu jeodezik mesafeye gore duzeltir.

    to_local_xy esdikdortgen bir yaklasimdir ve hedefin birkac yuz metre
    yakininda gecerlidir; 2.5 km yaricapta ~3 m sapma birakir. Kapi hem
    "hedefe 2500 m" olarak raporlanip hem de gecis denetimi jeodezik
    mesafeyle yapildigi icin iki olcunun ayni cemberi gostermesi gerekir.
    """
    def nokta(f: float) -> LatLon:
        return from_local_xy(
            start_xy[0] + f * (end_xy[0] - start_xy[0]),
            start_xy[1] + f * (end_xy[1] - start_xy[1]),
            center,
        )

    # Duzeltme izdusum hatasi kadar kucuktur; bacagin %2'lik komsulugu yeter.
    lo = max(0.0, fraction - 0.02)
    hi = min(1.0, fraction + 0.02)
    if geodesic_distance_m(nokta(lo), center) < radius_m:
        lo = 0.0
    if (
        geodesic_distance_m(nokta(lo), center) < radius_m
        or geodesic_distance_m(nokta(hi), center) > radius_m
    ):
        # Beklenen kusatma kurulamadi; duzlem cozumu bozmadan birakilir.
        return nokta(fraction)
    for _ in range(40):
        orta = 0.5 * (lo + hi)
        if geodesic_distance_m(nokta(orta), center) > radius_m:
            lo = orta
        else:
            hi = orta
    return nokta(0.5 * (lo + hi))


def last_circle_entry_on_route(
    home: LatLon,
    route: Sequence[LatLon],
    center: LatLon,
    radius_m: float,
) -> Optional[Tuple[LatLon, int]]:
    """Rotanin bir cembere son disaridan-iceri girisini bulur.

    Donen indeks, kesisimin bulundugu bacagin aktif waypoint indeksidir.
    Son girisin secilmesi onemlidir: rota korunan bolgeye girip yeniden
    cikiyorsa, bekleme icin geri donulemez son firsat daha sonraki giristir.
    """
    if not route or radius_m <= 0.0:
        return None

    result: Optional[Tuple[LatLon, int]] = None
    points = (home, *route)
    for active_index, (start, end) in enumerate(zip(points, points[1:])):
        start_xy = to_local_xy(start, center)
        # Baslangic zaten icerideyse bu bacak yeni bir giris olamaz.
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
