"""Jeodezik mesafe hesabi ve hedef kabul cemberi kesisim tespiti."""
from __future__ import annotations

import math
from typing import NamedTuple, Optional, Tuple

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
