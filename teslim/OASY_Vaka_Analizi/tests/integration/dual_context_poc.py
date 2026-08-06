#!/usr/bin/env python3
"""tek süreçte iki rclpy bağlamını ve alan ayrımını doğrular"""
from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from dataclasses import dataclass, field

import rclpy
from geographic_msgs.msg import GeoPoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

COORDINATION_DOMAIN_ID = 10  # ortak koordinasyon alanı
LEAK_PROBE_TOPIC = "/oasy_poc/leak_probe"  # alan sızıntısı denetim konusu
COORDINATION_TOPIC = "/oasy_poc/coordination"  # koordinasyon denetim konusu


@dataclass
class Counters:
    telemetry: int = 0
    coordination: int = 0
    leaked: int = 0
    last_latitude: float = 0.0
    errors: list = field(default_factory=list)


def build_vehicle_side(context: rclpy.Context, counters: Counters):
    """araç alanındaki telemetri ve sızıntı yayınını kurar"""
    node = rclpy.create_node("poc_vehicle", context=context)
    best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_geopose(msg: GeoPoseStamped) -> None:
        """gelen telemetri sayısını ve son enlemi kaydeder"""
        counters.telemetry += 1
        counters.last_latitude = msg.pose.position.latitude

    node.create_subscription(GeoPoseStamped, "/ap/geopose/filtered", on_geopose, best_effort)
    publisher = node.create_publisher(String, LEAK_PROBE_TOPIC, 10)
    node.create_timer(0.2, lambda: publisher.publish(String(data="arac domaini")))
    return node


def build_coordination_side(context: rclpy.Context, counters: Counters):
    """koordinasyon alanındaki yayın ve sızıntı dinleyicisini kurar"""
    node = rclpy.create_node("poc_coordination", context=context)
    best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

    node.create_subscription(
        String, COORDINATION_TOPIC,
        lambda _msg: setattr(counters, "coordination", counters.coordination + 1),
        best_effort,
    )
    node.create_subscription(
        String, LEAK_PROBE_TOPIC,
        lambda _msg: setattr(counters, "leaked", counters.leaked + 1),
        best_effort,
    )
    publisher = node.create_publisher(String, COORDINATION_TOPIC, best_effort)
    node.create_timer(0.2, lambda: publisher.publish(String(data="koordinasyon")))
    return node


def spin_in_thread(executor: SingleThreadedExecutor, counters: Counters) -> threading.Thread:
    """ros çalıştırıcısını ayrı bir thread içinde başlatır"""
    def run() -> None:
        """çalıştırıcıyı döndürür ve oluşan hatayı kaydeder"""
        try:
            executor.spin()
        except Exception as exc:  # noqa: BLE001
            counters.errors.append(f"{executor}: {exc}")

    thread = threading.Thread(target=run, daemon=False)
    thread.start()
    return thread


def main() -> int:
    """iki ros bağlamını kurup alan ayrımı denemesini çalıştırır"""
    parser = argparse.ArgumentParser(description="G1 cift-context dogrulamasi")
    parser.add_argument("--vehicle-domain", type=int, default=1)
    parser.add_argument("--seconds", type=float, default=12.0)
    args = parser.parse_args()

    counters = Counters()

    vehicle_context = rclpy.Context()
    rclpy.init(context=vehicle_context, domain_id=args.vehicle_domain,
               signal_handler_options=SignalHandlerOptions.NO)
    coordination_context = rclpy.Context()
    rclpy.init(context=coordination_context, domain_id=COORDINATION_DOMAIN_ID,
               signal_handler_options=SignalHandlerOptions.NO)

    vehicle_node = build_vehicle_side(vehicle_context, counters)
    coordination_node = build_coordination_side(coordination_context, counters)

    vehicle_executor = SingleThreadedExecutor(context=vehicle_context)
    vehicle_executor.add_node(vehicle_node)
    coordination_executor = SingleThreadedExecutor(context=coordination_context)
    coordination_executor.add_node(coordination_node)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    threads = [
        spin_in_thread(vehicle_executor, counters),
        spin_in_thread(coordination_executor, counters),
    ]

    print(f"arac domain={args.vehicle_domain}, koordinasyon domain={COORDINATION_DOMAIN_ID}, "
          f"{args.seconds:.0f} s calisacak")
    stop.wait(timeout=args.seconds)

    shutdown_start = time.monotonic()
    for executor in (vehicle_executor, coordination_executor):
        executor.shutdown()
    for node in (vehicle_node, coordination_node):
        node.destroy_node()
    for context in (vehicle_context, coordination_context):
        rclpy.shutdown(context=context)
    for thread in threads:
        thread.join(timeout=5.0)
    shutdown_s = time.monotonic() - shutdown_start

    return report(counters, threads, shutdown_s)


def report(counters: Counters, threads: list, shutdown_s: float) -> int:
    """alan ayrımı ve kapanış kontrollerinin sonucunu yazdırır"""
    alive = [t for t in threads if t.is_alive()]
    telemetry_ok = counters.telemetry > 0
    checks = {
        "koordinasyon context'i kendi yayinini aldi": counters.coordination > 0,
        "domain izolasyonu korundu": counters.leaked == 0,
        "thread'ler join oldu": not alive,
        "kapanista istisna yok": not counters.errors,
    }

    print("--- G1 sonuc ---")
    print(f"arac telemetrisi: {counters.telemetry} mesaj"
          f"{f' (son enlem {counters.last_latitude:.6f})' if telemetry_ok else ' (SITL yok, atlandi)'}")
    print(f"koordinasyon mesaji: {counters.coordination}")
    print(f"sizan mesaj: {counters.leaked} (0 olmali)")
    print(f"kapanis suresi: {shutdown_s:.2f} s")
    for name, passed in checks.items():
        print(f"  [{'GECTI' if passed else 'KALDI'}] {name}")
    for error in counters.errors:
        print(f"  hata: {error}")

    if not telemetry_ok:
        print("UYARI: SITL calismadigi icin telemetri sarti dogrulanmadi")
    print("SONUC:", "BASARILI" if all(checks.values()) else "BASARISIZ")
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
