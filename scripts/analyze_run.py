#!/usr/bin/env python3
"""görev koşusunun varış ölçümlerini toplar ve değerlendirir"""
from __future__ import annotations

import argparse
import json
import re
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional

import rclpy
from oasy_interfaces.msg import VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy

COORDINATION_TOPIC = "/oasy/vehicle_status"  # araç durumlarının yayınlandığı konu
EXPECTED_VEHICLE_IDS = (1, 2, 3)  # sonuçta beklenen araç kimlikleri
REQUIRED_SEPARATION_S = 20.0  # ardışık varışlar arasındaki hedef süre
SEPARATION_TOLERANCE_S = 1.0  # varış aralığı kabul toleransı
ARRIVAL_RADIUS_M = 5.0  # hedef kabul yarıçapı
MAX_ROUTE_DEVIATION_M = 500.0  # izin verilen en büyük rota sapması
NANOSECONDS_PER_SECOND = 1_000_000_000  # saniyedeki nanosaniye sayısı


@dataclass
class VehicleRecord:
    arrival_monotonic_ns: int = 0
    planned_arrival_monotonic_ns: int = 0
    seq_first: Optional[int] = None
    seq_last: int = 0
    message_count: int = 0

    @property
    def arrived(self) -> bool:
        """araç için geçerli bir varış anı kaydedilip kaydedilmediğini döner"""
        return self.arrival_monotonic_ns > 0

    @property
    def lost_messages(self) -> int:
        """sıra numaralarından tahmini kayıp mesaj sayısını hesaplar"""
        if self.seq_first is None:
            return 0
        expected = self.seq_last - self.seq_first + 1
        return max(expected - self.message_count, 0)


@dataclass
class RunCollector:
    records: Dict[int, VehicleRecord] = field(default_factory=dict)

    def update(self, msg: VehicleStatus) -> None:
        """gelen durum mesajıyla aracın koşu kaydını günceller"""
        record = self.records.setdefault(msg.vehicle_id, VehicleRecord())
        if record.seq_first is None:
            record.seq_first = msg.seq
        record.seq_last = max(record.seq_last, msg.seq)
        record.message_count += 1
        record.planned_arrival_monotonic_ns = msg.planned_arrival_monotonic_ns
        if msg.target_reached and not record.arrived:
            record.arrival_monotonic_ns = msg.actual_arrival_monotonic_ns

    @property
    def all_arrived(self) -> bool:
        """beklenen üç aracın da varıp varmadığını döner"""
        return all(
            vehicle_id in self.records and self.records[vehicle_id].arrived
            for vehicle_id in EXPECTED_VEHICLE_IDS
        )


def build_report(collector: RunCollector) -> dict:
    """toplanan varışlardan zaman farklarını ve sıra sonucunu oluşturur"""
    arrivals = {
        vehicle_id: record.arrival_monotonic_ns
        for vehicle_id, record in sorted(collector.records.items())
        if record.arrived
    }
    report: dict = {"varislar": {}, "farklar": {}, "sira_dogru": None}

    if not arrivals:
        return report

    first_ns = min(arrivals.values())
    for vehicle_id, arrival_ns in arrivals.items():
        record = collector.records[vehicle_id]
        planned_error_s = (
            (arrival_ns - record.planned_arrival_monotonic_ns) / NANOSECONDS_PER_SECOND
            if record.planned_arrival_monotonic_ns > 0 else None
        )
        report["varislar"][f"HA-{vehicle_id}"] = {
            "ilk_varisa_gore_s": round((arrival_ns - first_ns) / NANOSECONDS_PER_SECOND, 2),
            "plan_hatasi_s": round(planned_error_s, 2) if planned_error_s is not None else None,
            "kayip_mesaj": record.lost_messages,
        }

    ordered = sorted(arrivals.items())
    for (left_id, left_ns), (right_id, right_ns) in zip(ordered, ordered[1:]):
        delta_s = (right_ns - left_ns) / NANOSECONDS_PER_SECOND
        report["farklar"][f"HA-{right_id} - HA-{left_id}"] = {
            "fark_s": round(delta_s, 2),
            "sapma_s": round(delta_s - REQUIRED_SEPARATION_S, 2),
        }

    report["sira_dogru"] = [vid for vid, _ in ordered] == list(EXPECTED_VEHICLE_IDS) and all(
        arrivals[left] < arrivals[right]
        for left, right in zip(EXPECTED_VEHICLE_IDS, EXPECTED_VEHICLE_IDS[1:])
    )
    return report


_LOG_LINE = re.compile(  # araç ve günlük saatini ayıran kalıp
    r"agent_node-(?P<vehicle>[0-9])\]\s+(?P<clock>[0-9]{2}:[0-9]{2}:[0-9]{2})\s+HA-[0-9]"
)
_ARRIVAL = re.compile(  # hedef geçişi ve rota sapmasını ayıran kalıp
    r"en (?:yakın geçiş|yakin gecis) (?P<closest>[0-9.]+) m.*"
    r"max rota (?:sapması|sapmasi) (?P<deviation>[0-9]+) m"
)
_GROUND_WAIT = re.compile(r"yerde bekleme (?P<second>[0-9.]+) s")  # yerde bekleme kalıbı


def _clock_to_seconds(text: str) -> float:
    """saat metnini gün başından geçen saniyeye çevirir"""
    clock, minute, second = (int(part) for part in text.split(":"))
    return clock * 3600 + minute * 60 + second


def parse_log(path: Path) -> dict:
    """görev günlüğünden varış sapma ve bekleme ölçümlerini çıkarır"""
    result: dict = {}
    loiter_start: dict = {}
    for line in path.read_text(errors="replace").splitlines():
        head = _LOG_LINE.search(line)
        if head is None:
            continue
        vehicle = f"HA-{head.group('vehicle')}"
        entry = result.setdefault(
            vehicle,
            {"gecis_m": None, "max_sapma_m": None, "loiter_s": 0.0,
             "loiter_sayisi": 0, "yerde_bekleme_s": None},
        )
        moment = _clock_to_seconds(head.group("clock"))

        arrival = _ARRIVAL.search(line)
        if arrival is not None:
            entry["gecis_m"] = float(arrival.group("closest"))
            entry["max_sapma_m"] = float(arrival.group("deviation"))
        wait_s = _GROUND_WAIT.search(line)
        if wait_s is not None and (
            "taahhüt edildi" in line or "taahhut edildi" in line
        ):
            entry["yerde_bekleme_s"] = float(wait_s.group("second"))
        if "KAPI BEKLEMESİ BAŞLADI" in line or "KAPI BEKLEMESI BASLADI" in line:
            loiter_start[vehicle] = moment
        elif (
            "KAPI BEKLEMESİ BİTTİ" in line or "KAPI BEKLEMESI BITTI" in line
        ) and vehicle in loiter_start:
            entry["loiter_s"] += moment - loiter_start.pop(vehicle)
            entry["loiter_sayisi"] += 1
    return result


def evaluate(report: dict) -> dict:
    """koşu sonuçlarını kabul ölçütlerine göre değerlendirir"""
    reasons = []
    if not report.get("sira_dogru"):
        reasons.append("varış sırası yanlış")
    for label, metrics in report.get("farklar", {}).items():
        if abs(metrics["sapma_s"]) > SEPARATION_TOLERANCE_S:
            reasons.append(
                f"{label}: sapma {metrics['sapma_s']:+.2f} s "
                f"(sınır ±{SEPARATION_TOLERANCE_S:.1f} s)"
            )
    for label, metrics in report.get("gunluk", {}).items():
        closest = metrics.get("gecis_m")
        if closest is None:
            reasons.append(f"{label}: varış kaydı bulunamadı")
        elif closest > ARRIVAL_RADIUS_M:
            reasons.append(f"{label}: hedefe {closest:.2f} m (sınır {ARRIVAL_RADIUS_M:.0f} m)")
        deviation = metrics.get("max_sapma_m")
        if deviation is not None and deviation > MAX_ROUTE_DEVIATION_M:
            reasons.append(f"{label}: rota sapması {deviation:.0f} m (sınır {MAX_ROUTE_DEVIATION_M:.0f} m)")
    return {"gecti": not reasons, "nedenler": reasons}


def print_report(report: dict) -> None:
    """koşu raporunu okunabilir satırlar halinde ekrana yazar"""
    print("--- varış metrikleri ---")
    for name, values in report["varislar"].items():
        print(f"{name}: ilk varışa göre {values['ilk_varisa_gore_s']:+.2f} s | "
              f"plan hatası {values['plan_hatasi_s']} s | "
              f"kayıp mesaj {values['kayip_mesaj']}")
    for name, values in report["farklar"].items():
        print(f"{name}: {values['fark_s']:.2f} s (20 s'den sapma {values['sapma_s']:+.2f} s)")
    print(f"varış sırası HA-1/HA-2/HA-3: {'DOĞRU' if report['sira_dogru'] else 'YANLIŞ'}")

    log_stats = report.get("gunluk") or {}
    if log_stats:
        print("--- görev kuralları ---")
        for label, metrics in sorted(log_stats.items()):
            print(f"{label}: hedefe {metrics['gecis_m']} m | max rota sapması "
                  f"{metrics['max_sapma_m']} m | yerde bekleme "
                  f"{metrics['yerde_bekleme_s']} s | havada bekleme "
                  f"{metrics['loiter_s']:.0f} s ({metrics['loiter_sayisi']} kez)")
        total_loiter_s = sum(d["loiter_s"] for d in log_stats.values())
        total_ground_s = sum(d["yerde_bekleme_s"] or 0.0 for d in log_stats.values())
        # yerdeki ve havadaki toplam beklemeyi ayırır
        print(f"toplam bekleme: yerde {total_ground_s:.0f} s | havada {total_loiter_s:.0f} s")

    result = report.get("kabul") or {}
    if result:
        print(f"KABUL: {'GEÇTİ' if result['gecti'] else 'KALDI'}")
        for reason in result["nedenler"]:
            print(f"  - {reason}")


def main() -> int:
    """koşu analizini başlatır ve kabul sonucuyla çıkar"""
    parser = argparse.ArgumentParser(description="Gorev kosusu varis analizi")
    parser.add_argument("--timeout", type=float, default=1200.0)
    parser.add_argument("--domain", type=int, default=10)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--log", type=Path, default=None,
        help="agents.log yolu; 5 m varis, rota sapmasi ve bekleme sureleri "
             "yalnizca gunlukten okunabiliyor",
    )
    args = parser.parse_args()

    collector = RunCollector()
    rclpy.init(domain_id=args.domain)
    node = rclpy.create_node("oasy_run_analyzer")
    node.create_subscription(
        VehicleStatus, COORDINATION_TOPIC, collector.update,
        QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT),
    )

    print(f"domain {args.domain} dinleniyor, üç aracın varışı bekleniyor")
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and not collector.all_arrived:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

    report = build_report(collector)
    if args.log is not None and args.log.exists():
        report["gunluk"] = parse_log(args.log)
    report["kabul"] = evaluate(report)
    print_report(report)

    if args.output is not None:
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"rapor yazıldı: {args.output}")

    return 0 if report["kabul"]["gecti"] else 1


if __name__ == "__main__":
    sys.exit(main())
