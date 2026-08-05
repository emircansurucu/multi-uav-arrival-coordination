"""diğer araçların durumunu ve mesaj tazeliğini izler"""
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
    """diğer araçların geçerli durum mesajlarını saklar"""

    def __init__(self, own_vehicle_id: int, stale_after_s: float, lost_after_s: float) -> None:
        """araç kimliğini ve mesaj zaman aşımı sınırlarını hazırlar"""
        self._own_vehicle_id = own_vehicle_id
        self._stale_after_s = stale_after_s
        self._lost_after_s = lost_after_s
        self._lock = threading.Lock()
        self._peers: Dict[int, PeerRecord] = {}
        self.rejected_count = 0

    def update(self, status: VehicleStatus, received_monotonic_ns: int) -> bool:
        """yeni araç mesajını kaydeder kabul edilmediyse false döndürür"""
        if status.vehicle_id == self._own_vehicle_id:
            return False

        with self._lock:
            existing = self._peers.get(status.vehicle_id)
            # eski mesajın yeni durumu ezmesini önler
            if existing is not None and status.monotonic_ns < existing.status.monotonic_ns:
                self.rejected_count += 1
                return False
            self._peers[status.vehicle_id] = PeerRecord(status, received_monotonic_ns)
        return True

    def snapshot(self) -> Dict[int, PeerRecord]:
        """kayıtlı araç durumlarının güvenli bir kopyasını döner"""
        with self._lock:
            return dict(self._peers)

    def age_s(self, vehicle_id: int, now_monotonic_ns: int) -> float:
        """seçilen aracın son mesaj yaşını saniye olarak döner"""
        with self._lock:
            record = self._peers.get(vehicle_id)
        if record is None:
            return math.inf
        return (now_monotonic_ns - record.received_monotonic_ns) / 1e9

    def committed_arrivals(self, now_monotonic_ns: int) -> Dict[int, int]:
        """ulaşılabilen araçların taahhüt ettiği varış anlarını döndürür"""
        result: Dict[int, int] = {}
        for vehicle_id, record in self.snapshot().items():
            if not record.status.arrival_committed:
                continue
            if self.freshness(vehicle_id, now_monotonic_ns) is PeerFreshness.LOST:
                continue
            result[vehicle_id] = record.status.planned_arrival_monotonic_ns
        return result

    def feasible_arrivals(self, now_monotonic_ns: int) -> Dict[int, int]:
        """ulaşılabilen araçların en erken varış anlarını döndürür"""
        result: Dict[int, int] = {}
        for vehicle_id, record in self.snapshot().items():
            if record.status.earliest_feasible_arrival_monotonic_ns <= 0:
                continue
            if self.freshness(vehicle_id, now_monotonic_ns) is PeerFreshness.LOST:
                continue
            result[vehicle_id] = record.status.earliest_feasible_arrival_monotonic_ns
        return result

    def settled_wind(self, now_monotonic_ns: int) -> Optional[Tuple[float, float]]:
        """geçerli rüzgârı olan en küçük kimlikli aracın ölçümünü döndürür"""
        peers = self.snapshot()
        for vehicle_id in sorted(peers):
            record = peers[vehicle_id]
            if not record.status.wind_valid:
                continue
            if self.freshness(vehicle_id, now_monotonic_ns) is PeerFreshness.LOST:
                continue
            return (record.status.wind_speed, record.status.wind_dir_deg)
        return None

    def freshness(self, vehicle_id: int, now_monotonic_ns: int) -> PeerFreshness:
        """araç mesajını yaşına göre taze eski veya kayıp olarak sınıflandırır"""
        age = self.age_s(vehicle_id, now_monotonic_ns)
        if age >= self._lost_after_s:
            return PeerFreshness.LOST
        if age >= self._stale_after_s:
            return PeerFreshness.STALE
        return PeerFreshness.FRESH
