#!/usr/bin/env python3
"""Gorev kosusunun varis metriklerini olcer.

Koordinasyon domain'ini dinler, uc aracin da hedefe varmasini bekler ve
dokumandaki 20 saniyelik ardisik varis sartina gore sonucu raporlar.

Kullanim: scripts/analyze_run.py [--timeout 1200] [--domain 10]
"""
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

COORDINATION_TOPIC = "/oasy/vehicle_status"
EXPECTED_VEHICLE_IDS = (1, 2, 3)
# Vaka dokumani: ardisik varislar arasinda tam 20 saniye.
REQUIRED_SEPARATION_S = 20.0
# "Tam 20 saniye" pratikte bir tolerans gerektirir; olculen dagilim +-0.65 s
# oldugu icin kabul esigi 1 saniye secildi.
SEPARATION_TOLERANCE_S = 1.0
# Madde 2: hedefin kabul yaricapi 5 m. Madde 4: rotadan en fazla 500 m sapma.
ARRIVAL_RADIUS_M = 5.0
MAX_ROUTE_DEVIATION_M = 500.0
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


_LOG_SATIR = re.compile(
    r"agent_node-(?P<arac>[0-9])\]\s+(?P<saat>[0-9]{2}:[0-9]{2}:[0-9]{2})\s+HA-[0-9]"
)
_VARIS = re.compile(
    r"en yakin gecis (?P<gecis>[0-9.]+) m.*max rota sapmasi (?P<sapma>[0-9]+) m"
)
_YERDE_BEKLEME = re.compile(r"yerde bekleme (?P<saniye>[0-9.]+) s")


def _saat_saniye(metin: str) -> float:
    saat, dakika, saniye = (int(parca) for parca in metin.split(":"))
    return saat * 3600 + dakika * 60 + saniye


def parse_log(path: Path) -> dict:
    """agents.log'dan madde 4, 6 ve 8 kanitlarini cikarir.

    Bu degerler VehicleStatus uzerinden yayinlanmiyor (varis teshis verileri
    peer kararlarinda kullanilmadigi icin mesaj sade tutuldu), dolayisiyla
    tek kaynak gorev gunlugudur.
    """
    sonuc: dict = {}
    loiter_baslangic: dict = {}
    for satir in path.read_text(errors="replace").splitlines():
        basi = _LOG_SATIR.search(satir)
        if basi is None:
            continue
        arac = f"HA-{basi.group('arac')}"
        kayit = sonuc.setdefault(
            arac,
            {"gecis_m": None, "max_sapma_m": None, "loiter_s": 0.0,
             "loiter_sayisi": 0, "yerde_bekleme_s": None},
        )
        an = _saat_saniye(basi.group("saat"))

        varis = _VARIS.search(satir)
        if varis is not None:
            kayit["gecis_m"] = float(varis.group("gecis"))
            kayit["max_sapma_m"] = float(varis.group("sapma"))
        bekleme = _YERDE_BEKLEME.search(satir)
        if bekleme is not None and "taahhut edildi" in satir:
            kayit["yerde_bekleme_s"] = float(bekleme.group("saniye"))
        if "LOITER BASLADI" in satir:
            loiter_baslangic[arac] = an
        elif "LOITER BITTI" in satir and arac in loiter_baslangic:
            kayit["loiter_s"] += an - loiter_baslangic.pop(arac)
            kayit["loiter_sayisi"] += 1
    return sonuc


def evaluate(report: dict) -> dict:
    """Kabul olcutu: sira, 20 s toleransi, 5 m varis, 500 m sapma."""
    nedenler = []
    if not report.get("sira_dogru"):
        nedenler.append("varis sirasi yanlis")
    for ad, degerler in report.get("farklar", {}).items():
        if abs(degerler["sapma_s"]) > SEPARATION_TOLERANCE_S:
            nedenler.append(
                f"{ad}: sapma {degerler['sapma_s']:+.2f} s "
                f"(sinir +-{SEPARATION_TOLERANCE_S:.1f} s)"
            )
    for ad, degerler in report.get("gunluk", {}).items():
        gecis = degerler.get("gecis_m")
        if gecis is None:
            nedenler.append(f"{ad}: varis kaydi bulunamadi")
        elif gecis > ARRIVAL_RADIUS_M:
            nedenler.append(f"{ad}: hedefe {gecis:.2f} m (sinir {ARRIVAL_RADIUS_M:.0f} m)")
        sapma = degerler.get("max_sapma_m")
        if sapma is not None and sapma > MAX_ROUTE_DEVIATION_M:
            nedenler.append(f"{ad}: rota sapmasi {sapma:.0f} m (sinir {MAX_ROUTE_DEVIATION_M:.0f} m)")
    return {"gecti": not nedenler, "nedenler": nedenler}


def print_report(report: dict) -> None:
    print("--- varis metrikleri ---")
    for name, values in report["varislar"].items():
        print(f"{name}: ilk varisa gore {values['ilk_varisa_gore_s']:+.2f} s | "
              f"plan hatasi {values['plan_hatasi_s']} s | "
              f"kayip mesaj {values['kayip_mesaj']}")
    for name, values in report["farklar"].items():
        print(f"{name}: {values['fark_s']:.2f} s (20 s'den sapma {values['sapma_s']:+.2f} s)")
    print(f"varis sirasi HA-1/HA-2/HA-3: {'DOGRU' if report['sira_dogru'] else 'YANLIS'}")

    gunluk = report.get("gunluk") or {}
    if gunluk:
        print("--- gorev kurallari ---")
        for ad, degerler in sorted(gunluk.items()):
            print(f"{ad}: hedefe {degerler['gecis_m']} m | max rota sapmasi "
                  f"{degerler['max_sapma_m']} m | yerde bekleme "
                  f"{degerler['yerde_bekleme_s']} s | havada loiter "
                  f"{degerler['loiter_s']:.0f} s ({degerler['loiter_sayisi']} kez)")
        toplam_loiter = sum(d["loiter_s"] for d in gunluk.values())
        toplam_yerde = sum(d["yerde_bekleme_s"] or 0.0 for d in gunluk.values())
        # Madde 8: havada bekleme en az olmali. Yerde bekleme bedava,
        # havadaki loiter degil; oran ikisinin dengesini gosterir.
        print(f"toplam bekleme: yerde {toplam_yerde:.0f} s | havada {toplam_loiter:.0f} s")

    sonuc = report.get("kabul") or {}
    if sonuc:
        print(f"KABUL: {'GECTI' if sonuc['gecti'] else 'KALDI'}")
        for neden in sonuc["nedenler"]:
            print(f"  - {neden}")


def main() -> int:
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

    print(f"domain {args.domain} dinleniyor, uc aracin varisi bekleniyor...")
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
        print(f"rapor yazildi: {args.output}")

    return 0 if report["kabul"]["gecti"] else 1


if __name__ == "__main__":
    sys.exit(main())
