"""araçların merkeziyetsiz varış zamanlarını hesaplar"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

ARRIVAL_SEPARATION_S = 20.0  # ardışık varışlar arasındaki hedef süre
NANOSECONDS_PER_SECOND = 1_000_000_000  # saniyedeki nanosaniye sayısı


@dataclass(frozen=True)
class ReferenceArrival:
    """referans varış anını ve kaynak araçları tutar"""

    monotonic_ns: Optional[int]
    source_vehicle_ids: Tuple[int, ...]

    @property
    def resolved(self) -> bool:
        """referans varış anının belirlenip belirlenmediğini döner"""
        return self.monotonic_ns is not None


def compute_reference_arrival(
    vehicle_id: int,
    committed_arrivals: Dict[int, int],
    separation_s: float = ARRIVAL_SEPARATION_S,
) -> ReferenceArrival:
    """aracın hedeflemesi gereken mutlak varış anını hesaplar"""
    separation_ns = int(separation_s * NANOSECONDS_PER_SECOND)
    reference_ns: Optional[int] = None
    sources = []

    for peer_id, arrival_ns in sorted(committed_arrivals.items()):
        if peer_id >= vehicle_id:
            continue
        candidate = arrival_ns + separation_ns * (vehicle_id - peer_id)
        sources.append(peer_id)
        if reference_ns is None or candidate > reference_ns:
            reference_ns = candidate

    return ReferenceArrival(reference_ns, tuple(sources))


def compute_feasible_anchor(
    feasible_arrivals: Dict[int, int],
    separation_s: float = ARRIVAL_SEPARATION_S,
) -> Optional[int]:
    """bütün araçların ulaşabileceği ortak zamanlama çıpasını bulur"""
    anchor_ns: Optional[int] = None
    for vehicle_id, arrival_ns in feasible_arrivals.items():
        offset_ns = int((vehicle_id - 1) * separation_s * NANOSECONDS_PER_SECOND)
        candidate = arrival_ns - offset_ns
        if anchor_ns is None or candidate > anchor_ns:
            anchor_ns = candidate
    return anchor_ns


def target_arrival(
    anchor_ns: int,
    vehicle_id: int,
    separation_s: float = ARRIVAL_SEPARATION_S,
) -> int:
    """çıpadan aracın hedef varış anını hesaplar"""
    return anchor_ns + int((vehicle_id - 1) * separation_s * NANOSECONDS_PER_SECOND)


def compute_takeoff_time(reference_arrival_ns: int, nominal_flight_s: float) -> int:
    """referans varış için gereken kalkış anını hesaplar"""
    return reference_arrival_ns - int(nominal_flight_s * NANOSECONDS_PER_SECOND)



def compute_gate_release_window(
    target_arrival_ns: int,
    terminal_earliest_s: float,
    terminal_latest_s: float,
    early_margin_s: float = 0.0,
    late_margin_s: float = 0.0,
) -> Optional[Tuple[int, int]]:
    """hedeften son yasal kapının geçiş zaman aralığını hesaplar"""
    if target_arrival_ns <= 0 or terminal_earliest_s < 0.0 or terminal_latest_s < 0.0:
        return None
    if early_margin_s < 0.0 or late_margin_s < 0.0:
        raise ValueError("kapi marjlari negatif olamaz")

    lower_ns = target_arrival_ns - int(
        (terminal_latest_s - early_margin_s) * NANOSECONDS_PER_SECOND
    )
    upper_ns = target_arrival_ns - int(
        (terminal_earliest_s + late_margin_s) * NANOSECONDS_PER_SECOND
    )
    return lower_ns, upper_ns
