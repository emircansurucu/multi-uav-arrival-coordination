"""Hedefin kabul cemberine ilk girisinin tespiti.

Basari kriteri 5 m yaricapli cembere giristir. Telemetri ornekleri arasinda
kalan bir gecis kacirilmasin diye ardisik iki konum arasindaki dogru parcasi
cemberle kesistirilir; giris ani bu kesisimden interpolasyonla bulunur.

En yakin gecis mesafesi basari olcutu degildir; yalnizca metrik ve
basarisizlik teshisi icin tutulur.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

from .geodesy import LatLon, circle_entry_fraction, to_local_xy

DEFAULT_ARRIVAL_RADIUS_M = 5.0


class ArrivalDetector:
    """Varis olayini bir kez uretir; tekrarlanan tetiklemeyi engeller."""

    def __init__(self, target: LatLon, radius_m: float = DEFAULT_ARRIVAL_RADIUS_M) -> None:
        self._target = target
        self._radius_m = radius_m
        self._previous: Optional[Tuple[Tuple[float, float], int]] = None
        self.min_distance_m = math.inf
        self.arrival_monotonic_ns: Optional[int] = None
        self.interpolated = False

    @property
    def arrived(self) -> bool:
        return self.arrival_monotonic_ns is not None

    def update(self, position: LatLon, monotonic_ns: int) -> bool:
        """Yeni telemetri ornegini isler. Varis bu cagride olustuysa True doner."""
        if self.arrived:
            return False

        current_xy = to_local_xy(position, self._target)
        distance_m = math.hypot(*current_xy)
        self.min_distance_m = min(self.min_distance_m, distance_m)

        if self._previous is not None:
            previous_xy, previous_ns = self._previous
            fraction = circle_entry_fraction(previous_xy, current_xy, self._radius_m)
            if fraction is not None:
                self.arrival_monotonic_ns = previous_ns + int(
                    fraction * (monotonic_ns - previous_ns)
                )
                # Dogrudan cember icinde bir ornek yoksa giris ani turetilmistir.
                self.interpolated = distance_m > self._radius_m
                return True

        self._previous = (current_xy, monotonic_ns)
        return False
