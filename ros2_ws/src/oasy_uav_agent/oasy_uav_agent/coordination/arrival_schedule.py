"""Merkeziyetsiz varis zamanlamasi sozlesmesi.

Her arac kendi referans varis zamanini yalnizca peer'larin *taahhut edilmis*
degerlerinden hesaplar; hicbir arac digerine komut vermez.

Iki tasarim karari onemli:

  - Referans, goreli sure degil mutlak an olarak tasinir. "ETA + 20" bir
    suredir ve mesaj gecikmesiyle anlamini yitirir; mutlak monotonic zaman
    damgasi gecikmeye bagisiktir.
  - Yalnizca taahhut edilmis degerler kullanilir. Anlik ETA'ya baglanmak,
    onculun tahmin gurultusunu zincirleme buyutur ve hiz komutlarinda
    salinim yaratir.

Sira dokumanla sabittir: HA-1, HA-2, HA-3. Kucuk numarali arac oncedir ve
hicbir zaman buyuk numaraliyi takip etmez.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# Vaka dokumani: ardisik varislar arasinda tam 20 saniye.
ARRIVAL_SEPARATION_S = 20.0
NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True)
class ReferenceArrival:
    """Hesaplanmis referans varis ani ve hangi peer'lardan turedigi."""

    monotonic_ns: Optional[int]
    source_vehicle_ids: Tuple[int, ...]

    @property
    def resolved(self) -> bool:
        return self.monotonic_ns is not None


def compute_reference_arrival(
    vehicle_id: int,
    committed_arrivals: Dict[int, int],
    separation_s: float = ARRIVAL_SEPARATION_S,
) -> ReferenceArrival:
    """Aracin hedeflemesi gereken mutlak varis anini hesaplar.

    Her onceki arac j icin en az separation_s * (i - j) kadar sonra varilmali;
    kisitlarin en gec olani baglayicidir. Bir peer kaybolursa kalan peer'lardan
    turetilen kisit gecerli kalir.

    Oncu arac (en kucuk numara) icin referans yoktur; kendi nominal planini
    kullanir.
    """
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
    """Butun araclarin ulasabilecegi ortak zamanlama capasini bulur.

    Her arac icin capa adayi, o aracin en erken ulasabilecegi andan kendi
    sira gecikmesi cikarilarak bulunur. Baglayici olan en gec adaydir; yani
    capayi en yavas arac belirler.

    Ayni yayin verisini goren butun araclar ayni sonucu bagimsiz hesaplar,
    dolayisiyla merkezi bir karar noktasi olusmaz.
    """
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
    """Capadan aracin kendi hedef varis anini turetir."""
    return anchor_ns + int((vehicle_id - 1) * separation_s * NANOSECONDS_PER_SECOND)


def compute_takeoff_time(reference_arrival_ns: int, nominal_flight_s: float) -> int:
    """Referans varisa yetismek icin kalkisin yapilmasi gereken an."""
    return reference_arrival_ns - int(nominal_flight_s * NANOSECONDS_PER_SECOND)



def compute_gate_release_window(
    target_arrival_ns: int,
    terminal_earliest_s: float,
    terminal_latest_s: float,
    early_margin_s: float = 0.0,
    late_margin_s: float = 0.0,
) -> Optional[Tuple[int, int]]:
    """Hedeften, son yasal kapinin gecis zaman araligini turetir.

    Kapidan sonra loiter yoktur. E terminalde gec kalmamak icin gereken
    sureyi, L ise erken varisi yalniz hizla onleyebilecegimiz en uzun sureyi
    temsil eder. Donen aralik bos olabilir; bu, secilen bozucu modeli ve
    marjlar altinda kapidan sonraki kontrol yetkisinin yetersiz oldugunu
    acikca gosterir.
    """
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
