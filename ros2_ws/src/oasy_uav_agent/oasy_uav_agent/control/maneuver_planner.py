"""Zaman yedirme manevralarinin geometrisi.

Hiz kontrolu doygunluga ulastiginda (arac minimum hava hizinda oldugu halde
hala erken variyorsa) yol uzatmak gerekir. Iki kisit dokumandan gelir:

  - Hedefin 2 km cevresinde bekleme cemberi YASAK; orada yalnizca yorunge
    uzatma kullanilabilir.
  - Rotadan sapma en fazla 500 m olabilir.

S-manevrasinin olcusu su bagintidan cikar. Bacak uzunlugu L, eklenmek
istenen mesafe e ve tek bir zikzak dislisi icin yanal ofset d:

    2*sqrt((L/2)^2 + d^2) = L + e   =>   d = sqrt(e * (2L + e)) / 2

Bacagi N dongude bolmek hem L'yi hem e'yi bolduugu icin gereken yanal ofseti
hizla kucultur: 1304 m'lik bacakta +280 m icin tek dongu 450 m, uc dongu
150 m yanal sapma ister. Ust siniri sabit kanadin donus yaricapi koyar;
cok siki zikzagi otopilot kesip gecer ve planlanan mesafe kazanilmaz.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from ..estimation.geodesy import LatLon, from_local_xy, geodesic_distance_m, to_local_xy

# Dokuman madde 6: hedefin 2 km cevresinde loiter yasak.
LOITER_FORBIDDEN_RADIUS_M = 2000.0
# Dokuman madde 4: rotadan en fazla 500 m sapma.
MAX_LATERAL_OFFSET_M = 500.0
DEFAULT_MAX_CYCLES = 4
# Zikzagin bir dislisi, donusun tamamlanabilmesi icin donus capinin en az
# bu kati olmali; aksi halde otopilot koseyi keser ve mesafe kazanilmaz.
MIN_TOOTH_TO_DIAMETER_RATIO = 2.0


@dataclass(frozen=True)
class PathFollowState:
    """Yorunge takibinin anlik durumu."""

    carrot: LatLon
    segment_index: int
    remaining_to_end_m: float


@dataclass(frozen=True)
class SManeuver:
    """Uretilmis S-manevrasi ve gerceklesmesi beklenen kazanc."""

    waypoints: Tuple[LatLon, ...]
    lateral_offset_m: float
    cycles: int
    planned_extra_distance_m: float


def loiter_allowed(position: LatLon, target: LatLon) -> bool:
    """Bekleme cemberi bu konumda serbest mi (hedefe 2 km'den uzak mi)."""
    return geodesic_distance_m(position, target) > LOITER_FORBIDDEN_RADIUS_M


def lateral_offset_for_extra(leg_length_m: float, extra_m: float, cycles: int) -> float:
    """N dongulu zikzakta istenen ek mesafe icin gereken yanal ofset."""
    if leg_length_m <= 0.0 or extra_m <= 0.0 or cycles < 1:
        return 0.0
    tooth_length_m = leg_length_m / cycles
    tooth_extra_m = extra_m / cycles
    return math.sqrt(tooth_extra_m * (2.0 * tooth_length_m + tooth_extra_m)) / 2.0


def turn_radius_m(airspeed_mps: float, bank_angle_deg: float) -> float:
    """Sabit kanadin verilen banka acisindaki donus yaricapi."""
    return airspeed_mps ** 2 / (9.81 * math.tan(math.radians(bank_angle_deg)))


def plan_s_maneuver(
    start: LatLon,
    end: LatLon,
    extra_distance_m: float,
    min_turn_radius_m: float,
    max_lateral_offset_m: float = MAX_LATERAL_OFFSET_M,
    max_cycles: int = DEFAULT_MAX_CYCLES,
) -> Optional[SManeuver]:
    """start-end bacagini uzatan S-manevrasi uretir.

    Yanal sapmayi en aza indiren dongu sayisi secilir; donus yaricapinin
    izin verdiginden daha siki zikzak uretilmez. Kisitlar altinda hicbir
    cozum yoksa None doner ve cagiran taraf eksigi bilir.
    """
    leg_length_m = geodesic_distance_m(start, end)
    if leg_length_m <= 0.0 or extra_distance_m <= 0.0:
        return None

    minimum_tooth_m = MIN_TOOTH_TO_DIAMETER_RATIO * 2.0 * min_turn_radius_m
    best: Optional[Tuple[int, float]] = None
    for cycles in range(1, max_cycles + 1):
        if leg_length_m / cycles < minimum_tooth_m:
            break
        offset_m = lateral_offset_for_extra(leg_length_m, extra_distance_m, cycles)
        if offset_m > max_lateral_offset_m:
            continue
        if best is None or offset_m < best[1]:
            best = (cycles, offset_m)

    if best is None:
        return None

    cycles, offset_m = best
    return SManeuver(
        waypoints=_build_waypoints(start, end, cycles, offset_m),
        lateral_offset_m=offset_m,
        cycles=cycles,
        planned_extra_distance_m=extra_distance_m,
    )


def follow_path(
    path: Sequence[LatLon],
    position: LatLon,
    lookahead_m: float,
    min_segment_index: int = 0,
) -> Optional[PathFollowState]:
    """Yorunge uzerinde aracin onunde kayan takip noktasini bulur.

    ArduPlane GUIDED'da komut edilen nokta bir loiter merkezidir: arac oraya
    varinca WP_LOITER_RAD yaricapiyla cember atmaya baslar. Bu yuzden asla
    ulasilabilir bir nokta komut edilmez; hedef daima aracin onunde, takip
    mesafesi kadar ileride tutulur ve arac onu hicbir zaman yakalayamaz.

    min_segment_index geriye donusu engeller: bir segment gecildikten sonra
    aramaya daha erken segmentlerden baslanmaz.
    """
    if len(path) < 2 or lookahead_m <= 0.0:
        return None

    best: Optional[Tuple[int, float, float]] = None
    for index in range(min_segment_index, len(path) - 1):
        fraction, distance_m = _project_onto_segment(position, path[index], path[index + 1])
        if best is None or distance_m < best[2]:
            best = (index, fraction, distance_m)

    if best is None:
        return None

    segment_index, fraction, _ = best
    segment_length_m = geodesic_distance_m(path[segment_index], path[segment_index + 1])
    remaining_m = (1.0 - fraction) * segment_length_m
    for index in range(segment_index + 1, len(path) - 1):
        remaining_m += geodesic_distance_m(path[index], path[index + 1])

    # Takip noktasini bulunan izdusumden itibaren ileri dogru yurut.
    carrot = path[-1]
    cursor_index = segment_index
    cursor_fraction = fraction
    budget_m = lookahead_m
    while cursor_index < len(path) - 1:
        length_m = geodesic_distance_m(path[cursor_index], path[cursor_index + 1])
        available_m = (1.0 - cursor_fraction) * length_m
        if budget_m <= available_m:
            target_fraction = cursor_fraction + budget_m / length_m if length_m > 0 else 1.0
            carrot = _interpolate(path[cursor_index], path[cursor_index + 1], target_fraction)
            break
        budget_m -= available_m
        cursor_index += 1
        cursor_fraction = 0.0

    return PathFollowState(carrot, segment_index, remaining_m)


def _project_onto_segment(
    position: LatLon, start: LatLon, end: LatLon
) -> Tuple[float, float]:
    """Konumun segment uzerindeki izdusum orani ve dik uzakligi."""
    segment = to_local_xy(end, start)
    point = to_local_xy(position, start)
    squared = segment[0] ** 2 + segment[1] ** 2
    if squared == 0.0:
        return 0.0, math.hypot(*point)

    fraction = (point[0] * segment[0] + point[1] * segment[1]) / squared
    fraction = max(0.0, min(1.0, fraction))
    closest = (fraction * segment[0], fraction * segment[1])
    distance_m = math.hypot(point[0] - closest[0], point[1] - closest[1])
    return fraction, distance_m


def _interpolate(start: LatLon, end: LatLon, fraction: float) -> LatLon:
    segment = to_local_xy(end, start)
    return from_local_xy(segment[0] * fraction, segment[1] * fraction, start)


def _build_waypoints(
    start: LatLon, end: LatLon, cycles: int, offset_m: float
) -> Tuple[LatLon, ...]:
    """Bacak boyunca donusumlu yanal ofsetli ara noktalar uretir."""
    leg_east_m, leg_north_m = to_local_xy(end, start)
    leg_length_m = math.hypot(leg_east_m, leg_north_m)
    along = (leg_east_m / leg_length_m, leg_north_m / leg_length_m)
    # Bacak dogrultusunun soluna dik birim vektor.
    lateral = (-along[1], along[0])

    waypoints = []
    for index in range(cycles):
        fraction = (index + 0.5) / cycles
        sign = 1.0 if index % 2 == 0 else -1.0
        east_m = along[0] * leg_length_m * fraction + lateral[0] * offset_m * sign
        north_m = along[1] * leg_length_m * fraction + lateral[1] * offset_m * sign
        waypoints.append(from_local_xy(east_m, north_m, start))

    waypoints.append(end)
    return tuple(waypoints)
