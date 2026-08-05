"""hedef kabul çemberine ilk girişi tespit eder"""
from __future__ import annotations

import math
from typing import Optional, Tuple

from .geodesy import LatLon, circle_entry_fraction, to_local_xy

DEFAULT_ARRIVAL_RADIUS_M = 5.0  # varsayılan varış kabul yarıçapı


class ArrivalDetector:
    """varış olayını yalnızca ilk girişte üretir"""

    def __init__(self, target: LatLon, radius_m: float = DEFAULT_ARRIVAL_RADIUS_M) -> None:
        """hedefi ve varış kabul yarıçapını hazırlar"""
        self._target = target
        self._radius_m = radius_m
        self._previous: Optional[Tuple[Tuple[float, float], int]] = None
        self.min_distance_m = math.inf
        self.arrival_monotonic_ns: Optional[int] = None
        self.interpolated = False

    @property
    def arrived(self) -> bool:
        """varış olayının daha önce oluşup oluşmadığını döner"""
        return self.arrival_monotonic_ns is not None

    def update(self, position: LatLon, monotonic_ns: int) -> bool:
        """yeni konumu işler varış oluştuysa true döndürür"""
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
                # örnekler arasındaki çember girişini işaretler
                self.interpolated = distance_m > self._radius_m
                return True

        self._previous = (current_xy, monotonic_ns)
        return False
