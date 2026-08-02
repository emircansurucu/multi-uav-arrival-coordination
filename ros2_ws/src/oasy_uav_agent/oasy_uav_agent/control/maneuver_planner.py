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
    """Uretilmis S-manevrasi ve olculmus geometrisi.

    planned_extra_distance_m ve max_route_deviation_m istenen degerler degil,
    uretilen yorungenin uzerinde HESAPLANMIS degerlerdir. Onceki surumde
    istenen yanal ofset raporlaniyordu ve ucusta olculen sapma 9 kat cikmisti:
    manevra duz cizgi olarak planlanip rotayi kesiyordu, yani plan ile olcum
    ayni referansi kullanmiyordu.
    """

    waypoints: Tuple[LatLon, ...]
    lateral_offset_m: float
    cycles: int
    planned_extra_distance_m: float
    max_route_deviation_m: float


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


def polyline_length_m(points: Sequence[LatLon]) -> float:
    """Poligonun toplam uzunlugu."""
    return sum(
        geodesic_distance_m(start, end) for start, end in zip(points, points[1:])
    )


def distance_to_polyline_m(point: LatLon, polyline: Sequence[LatLon]) -> float:
    """Noktanin poligona en kisa uzakligi.

    Rota sapmasi bu olcuye gore tanimlanir: orijinal gorev rotasini olusturan
    sonlu segmentler kumesine minimum yatay uzaklik. Tek bir aktif bacaga gore
    olcmek, arac o bacagin disinda oldugunda boyuna mesafeyi de sapmaya
    katiyor ve degeri sisiriyor.
    """
    if len(polyline) < 2:
        return geodesic_distance_m(point, polyline[0]) if polyline else 0.0
    return min(
        _project_onto_segment(point, start, end)[1]
        for start, end in zip(polyline, polyline[1:])
    )


def _point_and_heading_at_arc(
    polyline: Sequence[LatLon], arc_m: float
) -> Tuple[LatLon, Tuple[float, float]]:
    """Poligon uzerinde verilen yay uzunlugundaki nokta ve yerel dogrultu."""
    kalan_m = arc_m
    for start, end in zip(polyline, polyline[1:]):
        length_m = geodesic_distance_m(start, end)
        if length_m <= 0.0:
            continue
        if kalan_m <= length_m:
            east_m, north_m = to_local_xy(end, start)
            unit = (east_m / length_m, north_m / length_m)
            return _interpolate(start, end, kalan_m / length_m), unit
        kalan_m -= length_m
    son_start, son_end = polyline[-2], polyline[-1]
    length_m = max(geodesic_distance_m(son_start, son_end), 1e-6)
    east_m, north_m = to_local_xy(son_end, son_start)
    return son_end, (east_m / length_m, north_m / length_m)


def _build_polyline_waypoints(
    polyline: Sequence[LatLon], cycles: int, offset_m: float
) -> Tuple[LatLon, ...]:
    """Poligonu izleyen, donusumlu yanal ofsetli yorunge uretir.

    Orijinal rota noktalari korunur ve ofset noktalari yay uzunluguna gore
    aralarina yerlestirilir. Boylece arac rotayi kesmez; onceki surum
    konumdan hedefe duz cizgi cizip aradaki waypoint'leri atliyordu.
    """
    total_m = polyline_length_m(polyline)
    # (yay konumu, nokta) ciftleri; orijinal noktalar ve disler birlikte siralanir.
    dugumler: list = []
    yay_m = 0.0
    for index, nokta in enumerate(polyline):
        if index > 0:
            yay_m += geodesic_distance_m(polyline[index - 1], nokta)
        dugumler.append((yay_m, nokta))

    for index in range(cycles):
        dis_yay_m = (index + 0.5) / cycles * total_m
        nokta, along = _point_and_heading_at_arc(polyline, dis_yay_m)
        yon = 1.0 if index % 2 == 0 else -1.0
        # Dogrultunun soluna dik birim vektor.
        lateral = (-along[1], along[0])
        dugumler.append((
            dis_yay_m,
            from_local_xy(lateral[0] * offset_m * yon, lateral[1] * offset_m * yon, nokta),
        ))

    dugumler.sort(key=lambda ikili: ikili[0])
    return tuple(nokta for _, nokta in dugumler)


def plan_s_maneuver(
    route: Sequence[LatLon],
    extra_distance_m: float,
    min_turn_radius_m: float,
    max_lateral_offset_m: float = MAX_LATERAL_OFFSET_M,
    max_cycles: int = DEFAULT_MAX_CYCLES,
) -> Optional[SManeuver]:
    """Kalan rota poligonunu uzatan S-manevrasi uretir.

    route[0] aracin mevcut konumu, route[-1] hedeftir; aradakiler henuz
    gecilmemis rota noktalaridir. Manevra bu poligonu izler, kesmez.

    Uretilen her aday icin gercek yol uzunlugu ve poligona gercek sapma
    hesaplanir; sapma sinirini asan adaylar elenir. Kisitlar altinda hicbir
    cozum yoksa None doner ve cagiran taraf manevradan vazgecer.
    """
    if len(route) < 2:
        return None
    total_m = polyline_length_m(route)
    if total_m <= 0.0 or extra_distance_m <= 0.0:
        return None

    minimum_tooth_m = MIN_TOOTH_TO_DIAMETER_RATIO * 2.0 * min_turn_radius_m
    best: Optional[SManeuver] = None
    for cycles in range(1, max_cycles + 1):
        if total_m / cycles < minimum_tooth_m:
            break
        offset_m = lateral_offset_for_extra(total_m, extra_distance_m, cycles)
        if offset_m > max_lateral_offset_m:
            continue

        waypoints = _build_polyline_waypoints(route, cycles, offset_m)
        deviation_m = max(distance_to_polyline_m(nokta, route) for nokta in waypoints)
        if deviation_m > max_lateral_offset_m:
            continue
        aday = SManeuver(
            waypoints=waypoints,
            lateral_offset_m=offset_m,
            cycles=cycles,
            planned_extra_distance_m=polyline_length_m(waypoints) - total_m,
            max_route_deviation_m=deviation_m,
        )
        if best is None or aday.max_route_deviation_m < best.max_route_deviation_m:
            best = aday

    return best


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


