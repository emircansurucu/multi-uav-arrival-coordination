"""Diger araclarin durum yayinlarinin tutulmasi ve tazeliginin izlenmesi."""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Tuple

from oasy_interfaces.msg import VehicleStatus


class PeerFreshness(Enum):
    FRESH = "taze"
    STALE = "eski"
    LOST = "kayip"


@dataclass(frozen=True)
class PeerRecord:
    status: VehicleStatus
    received_monotonic_ns: int


class PeerManager:
    """Peer durumlarini saklar; kendi yayinini ve geriye giden mesajlari eler."""

    def __init__(self, own_vehicle_id: int, stale_after_s: float, lost_after_s: float) -> None:
        self._own_vehicle_id = own_vehicle_id
        self._stale_after_s = stale_after_s
        self._lost_after_s = lost_after_s
        self._lock = threading.Lock()
        self._peers: Dict[int, PeerRecord] = {}
        self.rejected_count = 0

    def update(self, status: VehicleStatus, received_monotonic_ns: int) -> bool:
        """Yeni peer mesajini kaydeder. Kabul edilmediyse False doner."""
        if status.vehicle_id == self._own_vehicle_id:
            return False

        with self._lock:
            existing = self._peers.get(status.vehicle_id)
            # Sirasiz teslimde eski bir mesaj yeniyi ezmemeli.
            if existing is not None and status.monotonic_ns < existing.status.monotonic_ns:
                self.rejected_count += 1
                return False
            self._peers[status.vehicle_id] = PeerRecord(status, received_monotonic_ns)
        return True

    def snapshot(self) -> Dict[int, PeerRecord]:
        with self._lock:
            return dict(self._peers)

    def age_s(self, vehicle_id: int, now_monotonic_ns: int) -> float:
        with self._lock:
            record = self._peers.get(vehicle_id)
        if record is None:
            return math.inf
        return (now_monotonic_ns - record.received_monotonic_ns) / 1e9

    def committed_arrivals(self, now_monotonic_ns: int) -> Dict[int, int]:
        """Kaybolmamis peer'larin taahhut ettigi varis anlari.

        Taahhut edilmemis ya da kayip sayilan peer'lar disarida birakilir;
        boylece referans hesabi yalnizca guvenilir bilgiye dayanir.
        """
        result: Dict[int, int] = {}
        for vehicle_id, record in self.snapshot().items():
            if not record.status.arrival_committed:
                continue
            if self.freshness(vehicle_id, now_monotonic_ns) is PeerFreshness.LOST:
                continue
            result[vehicle_id] = record.status.planned_arrival_monotonic_ns
        return result

    def feasible_arrivals(self, now_monotonic_ns: int) -> Dict[int, int]:
        """Kaybolmamis peer'larin en erken ulasabilecegi varis anlari."""
        result: Dict[int, int] = {}
        for vehicle_id, record in self.snapshot().items():
            if record.status.earliest_feasible_arrival_monotonic_ns <= 0:
                continue
            if self.freshness(vehicle_id, now_monotonic_ns) is PeerFreshness.LOST:
                continue
            result[vehicle_id] = record.status.earliest_feasible_arrival_monotonic_ns
        return result

    def latest_wind(self, now_monotonic_ns: int) -> Optional[Tuple[float, float]]:
        """Kaybolmamis peer'lardan en tazesinin olctugu ruzgar (hiz, yon)."""
        newest_ns = -1
        result: Optional[Tuple[float, float]] = None
        for vehicle_id, record in self.snapshot().items():
            if not record.status.wind_valid:
                continue
            if self.freshness(vehicle_id, now_monotonic_ns) is PeerFreshness.LOST:
                continue
            if record.received_monotonic_ns > newest_ns:
                newest_ns = record.received_monotonic_ns
                result = (record.status.wind_speed, record.status.wind_dir_deg)
        return result

    def freshness(self, vehicle_id: int, now_monotonic_ns: int) -> PeerFreshness:
        age = self.age_s(vehicle_id, now_monotonic_ns)
        if age >= self._lost_after_s:
            return PeerFreshness.LOST
        if age >= self._stale_after_s:
            return PeerFreshness.STALE
        return PeerFreshness.FRESH
