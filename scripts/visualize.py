#!/usr/bin/env python3
"""koordinasyon alanını canlı harita ve durum panelinde gösterir"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple
from collections import deque

import yaml

import rclpy
from oasy_interfaces.msg import VehicleStatus
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy

import matplotlib
# ortamda seçilen çizim arka ucunu kullanır
matplotlib.use(os.environ.get("MPLBACKEND", "TkAgg"))
# gezinme çubuğunu gizler
matplotlib.rcParams["toolbar"] = "None"
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

PROJE_DIR = Path(__file__).resolve().parent.parent  # proje dizini
sys.path.insert(0, str(PROJE_DIR / "ros2_ws" / "src" / "oasy_uav_agent"))
from oasy_uav_agent.estimation.geodesy import LatLon, to_local_xy  # noqa: E402

COORDINATION_TOPIC = "/oasy/vehicle_status"  # araç durumlarının yayınlandığı konu
VEHICLE_IDS = (1, 2, 3)  # gösterilecek araç kimlikleri
NANOSECONDS_PER_SECOND = 1_000_000_000  # saniyedeki nanosaniye sayısı

TERMINAL_RADIUS_M = 2000.0  # terminal bölgesi yarıçapı
MIN_LOITER_DISTANCE_M = 2500.0  # son yasal bekleme mesafesi
ARRIVAL_RADIUS_M = 5.0  # hedef kabul yarıçapı
REQUIRED_SEPARATION_S = 20.0  # ardışık varışlar arasındaki hedef süre

VEHICLE_COLORS = {1: "#1f77b4", 2: "#d62728", 3: "#2ca02c"}  # araç çizim renkleri

STATE_NAMES = {  # panelde gösterilen görev durumu adları
    0: "Hazırlanıyor",
    1: "Bağlantı kuruluyor",
    2: "Rota hazırlanıyor",
    3: "Diğer araçlar bekleniyor",
    4: "Kalkış zamanı bekleniyor",
    5: "Kalkışa hazırlanıyor",
    6: "Kalkış",
    7: "Tırmanıyor",
    8: "Hedefe ilerliyor",
    9: "Hedefe yaklaşıyor",
    10: "Hedefe vardı",
    11: "Geri dönüyor",
    12: "Tamam",
    13: "Acil durum",
}

STATE_EVENTS = {  # görev durumu değişimlerinde gösterilen olaylar
    4: "HA-{v} planlanan kalkış zamanını bekliyor",
    5: "HA-{v} kalkışa hazırlanıyor",
    6: "HA-{v} kalkışa geçiyor",
    7: "HA-{v} 400 m irtifaya tırmanıyor",
    8: "HA-{v} hedefe doğru ilerliyor",
    9: "HA-{v} hedef bölgeye yaklaşıyor",
    10: "HA-{v} hedefe vardı",
    11: "HA-{v} kalkış noktasına geri dönüyor",
}

TRAIL_MAX_POINTS = 400  # araç izinde tutulacak en fazla nokta


@dataclass
class VehicleView:
    """tek aracın son durumunu ve iz geçmişini tutar"""

    msg: Optional[VehicleStatus] = None
    last_rx_wall: float = 0.0
    trail: Deque[Tuple[float, float]] = field(
        default_factory=lambda: deque(maxlen=TRAIL_MAX_POINTS)
    )
    last_state: Optional[int] = None


class DurumDeposu:
    """abonelik ve çizim iş parçacıkları arasındaki durumu tutar"""

    def __init__(self, origin: LatLon) -> None:
        """harita merkezini ve araç durum depolarını hazırlar"""
        self._lock = threading.Lock()
        self._origin = origin
        self.vehicles: Dict[int, VehicleView] = {v: VehicleView() for v in VEHICLE_IDS}
        self.events: Deque[Tuple[float, str]] = deque(maxlen=6)

    def guncelle(self, msg: VehicleStatus) -> None:
        """gelen araç mesajını iz ve olay bilgisiyle birlikte kaydeder"""
        with self._lock:
            view = self.vehicles.setdefault(msg.vehicle_id, VehicleView())
            view.msg = msg
            view.last_rx_wall = time.time()

            # araç izini kalkıştan sonra başlatır
            if msg.mission_state >= 6:
                x, y = to_local_xy(LatLon(msg.latitude, msg.longitude), self._origin)
                view.trail.append((x, y))

            if view.last_state != msg.mission_state:
                sablon = STATE_EVENTS.get(msg.mission_state)
                if sablon is not None:
                    self.events.append((time.time(), sablon.format(v=msg.vehicle_id)))
                view.last_state = msg.mission_state

    def anlik_kopya(self) -> Tuple[Dict[int, VehicleView], List[Tuple[float, str]]]:
        """araç ve olay bilgilerinin güvenli bir kopyasını döner"""
        with self._lock:
            return dict(self.vehicles), list(self.events)


def konfig_yukle(config_dir: Path) -> Dict[int, dict]:
    """üç aracın yaml ayarlarını kimliklerine göre yükler"""
    konfigler: Dict[int, dict] = {}
    for vehicle_id in VEHICLE_IDS:
        yol = config_dir / f"ha{vehicle_id}.yaml"
        with open(yol, encoding="utf-8") as fh:
            konfigler[vehicle_id] = yaml.safe_load(fh)
    return konfigler


def _rota_xy(konfig: dict, origin: LatLon) -> Tuple[List[float], List[float]]:
    """araç rotasını harita merkezine göre yerel koordinatlara çevirir"""
    noktalar = [LatLon(konfig["home"]["lat"], konfig["home"]["lon"])]
    noktalar += [LatLon(p["lat"], p["lon"]) for p in konfig["route"]]
    xy = [to_local_xy(p, origin) for p in noktalar]
    return [p[0] for p in xy], [p[1] for p in xy]


class Gorsellestirici:
    """canlı harita durum paneli ve olay akışını çizer"""

    def __init__(self, konfigler: Dict[int, dict], depo: DurumDeposu, origin: LatLon):
        """harita panel ve olay alanlarını oluşturur"""
        self._konfigler = konfigler
        self._depo = depo
        self._origin = origin
        self._t0 = time.time()

        self.fig = plt.figure(figsize=(16, 9), facecolor="#111417")
        gs = self.fig.add_gridspec(
            2, 2, width_ratios=[2.20, 1.0], height_ratios=[5.2, 1.0],
            left=0.025, right=0.992, top=0.94, bottom=0.04, wspace=0.08, hspace=0.14,
        )
        self._ax_map = self.fig.add_subplot(gs[0, 0])
        self._ax_panel = self.fig.add_subplot(gs[0, 1])
        self._ax_ticker = self.fig.add_subplot(gs[1, :])

        self.fig.suptitle(
            "OASY üç araçlı uçuş takibi  |  HA-1 → HA-2 → HA-3, 20 s aralık",
            color="#e8eaed", fontsize=17, fontweight="bold",
        )
        self._harita_kur()
        self._panel_kur()
        # tam ekranda klavyeyle çıkış sağlar
        self.fig.canvas.mpl_connect("key_press_event", self._tusa_basildi)

    @staticmethod
    def _tusa_basildi(olay) -> None:
        """kaçış veya q tuşuyla çizim penceresini kapatır"""
        if olay.key in ("escape", "q"):
            plt.close("all")

    # sabit harita katmanı

    def _harita_kur(self) -> None:
        """sabit rotaları hedef bölgelerini ve araç katmanlarını hazırlar"""
        ax = self._ax_map
        ax.set_facecolor("#181c20")
        for kenar in ax.spines.values():
            kenar.set_color("#3a4148")
        ax.tick_params(colors="#8b949e", labelsize=9)
        ax.grid(True, color="#262c32", linewidth=0.6)
        # eşit ölçeği koruyarak harita alanını doldurur
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("doğu (m)", color="#8b949e", fontsize=10)
        ax.set_ylabel("kuzey (m)", color="#8b949e", fontsize=10)

        tum_x: List[float] = []
        tum_y: List[float] = []
        for vehicle_id, konfig in self._konfigler.items():
            xs, ys = _rota_xy(konfig, self._origin)
            tum_x += xs
            tum_y += ys
            renk = VEHICLE_COLORS[vehicle_id]
            ax.plot(xs, ys, "--", color=renk, linewidth=1.6, alpha=0.55, zorder=2)
            ax.plot(xs[1:-1], ys[1:-1], "o", color=renk, markersize=5.0, alpha=0.75, zorder=3)
            ax.plot(xs[0], ys[0], "s", color=renk, markersize=10.5,
                    markeredgecolor="#e8eaed", markeredgewidth=0.8, zorder=4)
            ax.annotate(f"HA-{vehicle_id} pisti", (xs[0], ys[0]),
                        textcoords="offset points", xytext=(9, -12),
                        color=renk, fontsize=11, fontweight="bold")

        # terminal ve son yasal bekleme çemberlerini çizer
        ax.add_patch(Circle((0, 0), TERMINAL_RADIUS_M, fill=False, linestyle="--",
                            edgecolor="#e5534b", linewidth=1.8, alpha=0.9, zorder=5))
        ax.add_patch(Circle((0, 0), TERMINAL_RADIUS_M, facecolor="#e5534b",
                            alpha=0.06, zorder=1))
        ax.add_patch(Circle((0, 0), MIN_LOITER_DISTANCE_M, fill=False, linestyle=":",
                            edgecolor="#d29922", linewidth=1.6, alpha=0.9, zorder=5))
        ax.plot(0, 0, "*", color="#e8eaed", markersize=23, zorder=6)

        # harita açıklamalarını sabit köşede gösterir
        gosterge = (
            ("★  ortak hedef  5 m", "#e8eaed", "bold"),
            ("--  havada bekleme YASAK  2 km", "#e5534b", "bold"),
            ("··  bekleme sınırı  2.5 km", "#d29922", "normal"),
            ("■  kalkış pisti", "#8b949e", "normal"),
        )
        for sira, (metin, renk, kalinlik) in enumerate(gosterge):
            ax.text(0.014, 0.978 - sira * 0.036, metin, transform=ax.transAxes,
                    color=renk, fontsize=11, fontweight=kalinlik,
                    va="top", ha="left", family="monospace", zorder=11)

        marj = 500.0
        ax.set_xlim(min(tum_x) - marj, max(tum_x) + marj)
        ax.set_ylim(min(tum_y) - marj, max(tum_y) + marj)

        # hedef çevresini ayrı büyüteçte gösterir
        self._ax_zoom = ax.inset_axes([0.015, 0.02, 0.32, 0.32])
        self._buyutec_kur()

        # araç izlerini gövdelerini ve etiketlerini hazırlar
        etiket_kaydirma = {1: (12, 9, "left"), 2: (12, -20, "left"), 3: (-12, 9, "right")}
        self._iz_cizgi = {}
        self._govde = {}
        self._etiket = {}
        self._iz_zoom = {}
        self._govde_zoom = {}
        for vehicle_id in VEHICLE_IDS:
            renk = VEHICLE_COLORS[vehicle_id]
            dx, dy, hiza = etiket_kaydirma[vehicle_id]
            (self._iz_cizgi[vehicle_id],) = ax.plot(
                [], [], "-", color=renk, linewidth=3.4, alpha=0.95, zorder=7)
            (self._govde[vehicle_id],) = ax.plot(
                [], [], "^", color=renk, markersize=20,
                markeredgecolor="#e8eaed", markeredgewidth=1.3, zorder=8)
            self._etiket[vehicle_id] = ax.annotate(
                "", (0, 0), textcoords="offset points", xytext=(dx, dy), ha=hiza,
                color=renk, fontsize=12, fontweight="bold", zorder=9)
            (self._iz_zoom[vehicle_id],) = self._ax_zoom.plot(
                [], [], "-", color=renk, linewidth=2.7, alpha=0.95, zorder=7)
            (self._govde_zoom[vehicle_id],) = self._ax_zoom.plot(
                [], [], "^", color=renk, markersize=15,
                markeredgecolor="#e8eaed", markeredgewidth=1.1, zorder=8)

        self._ruzgar_ok = ax.annotate(
            "", xy=(0, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="-|>", color="#58a6ff", linewidth=2.7), zorder=10)
        self._ruzgar_yazi = ax.text(
            0.98, 0.97, "", transform=ax.transAxes, color="#58a6ff",
            fontsize=11, fontweight="bold", ha="right", va="top", zorder=10,
            bbox=dict(facecolor="#181c20", edgecolor="none", alpha=0.8, pad=2))

    def _buyutec_kur(self) -> None:
        """hedef çevresini yakından gösteren ekseni kurar"""
        ax = self._ax_zoom
        # hedef çevresinde üç aracı ayıracak ölçeği kullanır
        yari = 1500.0
        ax.set_xlim(-yari, yari)
        ax.set_ylim(-yari, yari)
        ax.set_aspect("equal", adjustable="box")
        ax.set_facecolor("#101418")
        ax.set_xticks([])
        ax.set_yticks([])
        for kenar in ax.spines.values():
            kenar.set_color("#8b949e")
            kenar.set_linewidth(1.2)

        for konfig in self._konfigler.values():
            xs, ys = _rota_xy(konfig, self._origin)
            ax.plot(xs, ys, "--", color="#4a545e", linewidth=0.9, zorder=2)

        ax.add_patch(Circle((0, 0), TERMINAL_RADIUS_M, fill=False, linestyle="--",
                            edgecolor="#e5534b", linewidth=1.2, alpha=0.8, zorder=3))
        # mesafe karşılaştırması için 500 metre halkası çizer
        ax.add_patch(Circle((0, 0), 500.0, fill=False, linestyle=":",
                            edgecolor="#3a4148", linewidth=0.9, zorder=3))
        ax.plot(0, 0, "*", color="#e8eaed", markersize=15, zorder=6)
        ax.set_title("hedef yakını  ±1.5 km   (nokta halka: 500 m)",
                     color="#8b949e", fontsize=9.2, pad=4)

    def _panel_kur(self) -> None:
        """araç durum ve olay metinlerinin panel alanlarını hazırlar"""
        for ax in (self._ax_panel, self._ax_ticker):
            ax.set_facecolor("#181c20")
            ax.set_xticks([])
            ax.set_yticks([])
            for kenar in ax.spines.values():
                kenar.set_color("#3a4148")

        self._panel_yazi = self._ax_panel.text(
            0.035, 0.975, "", transform=self._ax_panel.transAxes,
            va="top", ha="left", color="#e8eaed", fontsize=13,
            family="monospace", linespacing=1.28)
        self._ticker_yazi = self._ax_ticker.text(
            0.012, 0.88, "", transform=self._ax_ticker.transAxes,
            va="top", ha="left", color="#e8eaed", fontsize=13,
            family="monospace", linespacing=1.25)

    # canlı araç katmanı

    def guncelle(self, _frame: int):
        """son araç durumlarıyla bütün canlı çizim katmanlarını yeniler"""
        # ros kapandığında çizim penceresini de kapatır
        if not rclpy.ok():
            plt.close("all")
            return []
        araclar, olaylar = self._depo.anlik_kopya()
        self._haritayi_ciz(araclar)
        self._paneli_ciz(araclar)
        self._tickeri_ciz(olaylar)
        return []

    def _haritayi_ciz(self, araclar: Dict[int, VehicleView]) -> None:
        """araç izlerini konumlarını ve etiketlerini haritada günceller"""
        for vehicle_id in VEHICLE_IDS:
            view = araclar.get(vehicle_id)
            iz = self._iz_cizgi[vehicle_id]
            govde = self._govde[vehicle_id]
            etiket = self._etiket[vehicle_id]

            iz_zoom = self._iz_zoom[vehicle_id]
            govde_zoom = self._govde_zoom[vehicle_id]

            if view is None or view.msg is None or not view.trail:
                iz.set_data([], [])
                govde.set_data([], [])
                iz_zoom.set_data([], [])
                govde_zoom.set_data([], [])
                etiket.set_text("")
                continue

            xs = [p[0] for p in view.trail]
            ys = [p[1] for p in view.trail]
            iz.set_data(xs, ys)
            govde.set_data([xs[-1]], [ys[-1]])
            iz_zoom.set_data(xs, ys)
            govde_zoom.set_data([xs[-1]], [ys[-1]])

            # burun yönünü ardışık konumlardan çıkarır
            if len(xs) >= 2:
                aci = math.degrees(math.atan2(ys[-1] - ys[-2], xs[-1] - xs[-2]))
                govde.set_marker((3, 0, aci - 90.0))
                govde_zoom.set_marker((3, 0, aci - 90.0))

            # konum .xy ile taşınır çünkü etiket uzaklığı offset points cinsinden
            etiket.xy = (xs[-1], ys[-1])
            etiket.set_text(f"HA-{vehicle_id} · {view.msg.groundspeed:.0f} m/s")

        self._ruzgari_ciz(araclar)

    def _ruzgari_ciz(self, araclar: Dict[int, VehicleView]) -> None:
        """geçerli rüzgâr ölçümünü yön oku ve metinle gösterir"""
        # geçerli rüzgâr bildiren ilk aracı kullanır
        ruzgar = None
        for vehicle_id in VEHICLE_IDS:
            view = araclar.get(vehicle_id)
            if view and view.msg and view.msg.wind_valid and view.msg.wind_speed > 0.1:
                ruzgar = view.msg
                break
        if ruzgar is None:
            self._ruzgar_ok.set_visible(False)
            self._ruzgar_yazi.set_text("")
            return

        x0, x1 = self._ax_map.get_xlim()
        y0, y1 = self._ax_map.get_ylim()
        # rüzgâr göstergesini sağ üste yerleştirir
        taban_x = x0 + (x1 - x0) * 0.87
        taban_y = y1 - (y1 - y0) * 0.13
        # oku rüzgârın gittiği yöne çizer
        gidis = math.radians(ruzgar.wind_dir_deg + 180.0)
        uzunluk = (x1 - x0) * 0.055
        self._ruzgar_ok.set_visible(True)
        self._ruzgar_ok.xy = (taban_x + uzunluk * math.sin(gidis),
                              taban_y + uzunluk * math.cos(gidis))
        self._ruzgar_ok.set_position((taban_x, taban_y))
        self._ruzgar_yazi.set_text(
            f"RÜZGÂR  {ruzgar.wind_speed:.1f} m/s  {ruzgar.wind_dir_deg:.0f}°'den")

    def _paneli_ciz(self, araclar: Dict[int, VehicleView]) -> None:
        """araçların canlı görev ve telemetri değerlerini panele yazar"""
        simdi = time.time()
        # görev yerine izleme süresini gösterir
        satirlar = [f"  izleme       {simdi - self._t0:7.1f} s", ""]

        for vehicle_id in VEHICLE_IDS:
            view = araclar.get(vehicle_id)
            satirlar.append(f"  ── HA-{vehicle_id} ──────────────────")
            if view is None or view.msg is None:
                satirlar += ["     (yayın yok)", ""]
                continue

            m = view.msg
            bayat = simdi - view.last_rx_wall
            tazelik = "" if bayat < 1.5 else f"  [bayat {bayat:.0f}s]"
            satirlar.append(f"   durum  {STATE_NAMES.get(m.mission_state, '?')}{tazelik}")
            satirlar.append(f"   irtifa {m.altitude_msl:7.0f} m")
            satirlar.append(f"   yer hızı{m.groundspeed:6.1f} m/s")
            # hesaplanmamış mesafe ve varış süresini boş gösterir
            if m.remaining_distance > 0.0:
                satirlar.append(f"   kalan  {m.remaining_distance / 1000.0:7.2f} km")
            else:
                satirlar.append("   kalan      yok")
            if m.target_reached:
                satirlar.append("   varış    tamam")
            elif m.eta_seconds > 0.0:
                satirlar.append(f"   varış  {m.eta_seconds:7.1f} s")
            else:
                satirlar.append("   varış      yok")
            satirlar.append("")

        satirlar += self._zamanlama_satirlari(araclar)
        self._panel_yazi.set_text("\n".join(satirlar))

    def _zamanlama_satirlari(self, araclar: Dict[int, VehicleView]) -> List[str]:
        """planlanan ve gerçek varış aralıklarını gösterir"""
        satirlar = ["  ── VARIŞ ARALIĞI ─────────", ""]

        planli: Dict[int, int] = {}
        gercek: Dict[int, int] = {}
        for vehicle_id in VEHICLE_IDS:
            view = araclar.get(vehicle_id)
            if view is None or view.msg is None:
                continue
            if view.msg.planned_arrival_monotonic_ns > 0:
                planli[vehicle_id] = view.msg.planned_arrival_monotonic_ns
            if view.msg.target_reached and view.msg.actual_arrival_monotonic_ns > 0:
                gercek[vehicle_id] = view.msg.actual_arrival_monotonic_ns

        def _fark(kaynak: Dict[int, int], onceki: int, sonraki: int) -> Optional[float]:
            """iki aracın kayıtlı varış zamanı farkını saniye olarak döner"""
            if onceki in kaynak and sonraki in kaynak:
                return (kaynak[sonraki] - kaynak[onceki]) / NANOSECONDS_PER_SECOND
            return None

        for onceki, sonraki in ((1, 2), (2, 3)):
            etiket = f"   HA-{onceki}→HA-{sonraki}"
            g = _fark(gercek, onceki, sonraki)
            if g is not None:
                sapma = g - REQUIRED_SEPARATION_S
                satirlar.append(f"{etiket}  {g:6.2f} s  ({sapma:+.2f})  GERÇEK")
                continue
            p = _fark(planli, onceki, sonraki)
            if p is not None:
                satirlar.append(f"{etiket}  {p:6.2f} s   planlı")
            else:
                satirlar.append(f"{etiket}     yok")
        return satirlar

    def _tickeri_ciz(self, olaylar: List[Tuple[float, str]]) -> None:
        """son görev olaylarını alt bilgi alanında gösterir"""
        if not olaylar:
            self._ticker_yazi.set_text("  olay bekleniyor")
            return
        satirlar = []
        for zaman, metin in list(olaylar)[-4:]:
            satirlar.append(f"  {time.strftime('%H:%M:%S', time.localtime(zaman))}   {metin}")
        self._ticker_yazi.set_text("\n".join(satirlar))


def main() -> int:
    """ros aboneliğini ve canlı görev görselleştirmesini başlatır"""
    ayristirici = argparse.ArgumentParser(description=__doc__)
    ayristirici.add_argument("--domain", type=int, default=10,
                             help="koordinasyon DDS domain kimligi")
    ayristirici.add_argument("--config-dir", type=Path,
                             default=PROJE_DIR / "ros2_ws" / "src" / "oasy_bringup" / "config")
    ayristirici.add_argument("--fps", type=float, default=5.0,
                             help="cizim tazeleme hizi; yayin zaten ~5 Hz")
    ayristirici.add_argument("--snapshot", type=Path, default=None,
                             help="tek kare PNG uretip cik (ekransiz dogrulama icin)")
    ayristirici.add_argument("--window", default=None, metavar="WxH+X+Y",
                             help="pencereyi kayit cercevesine oturt, orn 1920x1080+0+0")
    ayristirici.add_argument("--fullscreen", action="store_true",
                             help="tam ekran; ekran kaydinda masaustu cubuklarini disarida birakir")
    args = ayristirici.parse_args()

    konfigler = konfig_yukle(args.config_dir)
    # ortak hedefi harita merkezi yapar
    son = konfigler[1]["route"][-1]
    origin = LatLon(son["lat"], son["lon"])

    depo = DurumDeposu(origin)

    rclpy.init(domain_id=args.domain)
    node = rclpy.create_node("oasy_visualizer")
    node.create_subscription(
        VehicleStatus, COORDINATION_TOPIC, depo.guncelle,
        QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT))

    # ros aboneliğini ayrı iş parçacığında çalıştırır
    durduruldu = threading.Event()

    def _don() -> None:
        """ros mesajlarını arka planda işlemeye devam eder"""
        # ros kapanış istisnasını sessizce karşılar
        try:
            while not durduruldu.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=_don, daemon=True)
    thread.start()

    gorsel = Gorsellestirici(konfigler, depo, origin)
    try:
        if args.snapshot is not None:
            # tek kareden önce birkaç durum mesajı bekler
            time.sleep(2.0)
            gorsel.guncelle(0)
            gorsel.fig.savefig(args.snapshot, dpi=100, facecolor=gorsel.fig.get_facecolor())
            print(f"kare yazildi: {args.snapshot}")
        else:
            # animasyon referansını canlı tutar
            gorsel.anim = FuncAnimation(gorsel.fig, gorsel.guncelle,
                                        interval=int(1000.0 / args.fps), blit=False,
                                        cache_frame_data=False)
            # kayıt için pencereyi tam ekrana alır
            try:
                if args.fullscreen:
                    gorsel.fig.canvas.manager.full_screen_toggle()
                elif args.window:
                    gorsel.fig.canvas.manager.window.wm_geometry(args.window)
            except AttributeError:
                print("uyari: bu backend pencere denetimini desteklemiyor",
                      file=sys.stderr)
            plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        durduruldu.set()
        thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
