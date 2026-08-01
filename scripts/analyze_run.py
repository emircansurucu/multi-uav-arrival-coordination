#!/usr/bin/env python3
"""Gorev kosusunun varis metriklerini olcer.

Koordinasyon domain'ini dinler, uc aracin da hedefe varmasini bekler ve
dokumandaki 20 saniyelik ardisik varis sartina gore sonucu raporlar.

Kullanim: scripts/analyze_run.py [--timeout 1200] [--domain 10]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional

import rclpy
from oasy_interfaces.msg import VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy

COORDINATION_TOPIC = "/oasy/vehicle_status"
EXPECTED_VEHICLE_IDS = (1, 2, 3)
# Vaka dokumani: ardisik varislar arasinda tam 20 saniye.
REQUIRED_SEPARATION_S = 20.0
NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass
class VehicleRecord:
    arrival_monotonic_ns: int = 0
    planned_arrival_monotonic_ns: int = 0
    seq_first: Optional[int] = None
    seq_last: int = 0
    message_count: int = 0

    @property
    def arrived(self) -> bool:
        return self.arrival_monotonic_ns > 0

    @property
    def lost_messages(self) -> int:
        if self.seq_first is None:
            return 0
        expected = self.seq_last - self.seq_first + 1
        return max(expected - self.message_count, 0)


@dataclass
class RunCollector:
    records: Dict[int, VehicleRecord] = field(default_factory=dict)

    def update(self, msg: VehicleStatus) -> None:
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
        return all(
            vehicle_id in self.records and self.records[vehicle_id].arrived
            for vehicle_id in EXPECTED_VEHICLE_IDS
        )


def build_report(collector: RunCollector) -> dict:
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


def print_report(report: dict) -> None:
    print("--- varis metrikleri ---")
    for name, values in report["varislar"].items():
        print(f"{name}: ilk varisa gore {values['ilk_varisa_gore_s']:+.2f} s | "
              f"plan hatasi {values['plan_hatasi_s']} s | "
              f"kayip mesaj {values['kayip_mesaj']}")
    for name, values in report["farklar"].items():
        print(f"{name}: {values['fark_s']:.2f} s (20 s'den sapma {values['sapma_s']:+.2f} s)")
    print(f"varis sirasi HA-1/HA-2/HA-3: {'DOGRU' if report['sira_dogru'] else 'YANLIS'}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Gorev kosusu varis analizi")
    parser.add_argument("--timeout", type=float, default=1200.0)
    parser.add_argument("--domain", type=int, default=10)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    collector = RunCollector()
    rclpy.init(domain_id=args.domain)
    node = rclpy.create_node("oasy_run_analyzer")
    node.create_subscription(
        VehicleStatus, COORDINATION_TOPIC, collector.update,
        QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT),
    )

    print(f"domain {args.domain} dinleniyor, uc aracin varisi bekleniyor...")
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and not collector.all_arrived:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

    report = build_report(collector)
    print_report(report)

    if args.output is not None:
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"rapor yazildi: {args.output}")

    return 0 if collector.all_arrived else 1


if __name__ == "__main__":
    sys.exit(main())
