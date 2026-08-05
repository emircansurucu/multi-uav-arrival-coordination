#!/usr/bin/env python3
"""ham ekran kaydını altyazılı ve hızlandırılmış videoya çevirir"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

SEYIR_HIZI = 20.0  # altyazı dışındaki bölümlerin hız çarpanı
BASLIK_SURE_S = 5.0  # açılış başlığının ekranda kalma süresi
OLAY_SURE_S = 4.0  # normal olay altyazısının ekranda kalma süresi
BASARI_SURE_S = 5.5  # önemli olay altyazısının ekranda kalma süresi
RTL_ALTYAZI_GECIKMESI_S = 30.0  # dönüş bilgi altyazısı gecikmesi
GRUPLAMA_PENCERESI_S = 1.5  # aynı blokta gösterilecek olay aralığı
HEDEF_G = 1920  # çıktı genişliği
HEDEF_Y = 1080  # çıktı yüksekliği

DURUM_SATIRI = re.compile(  # görev durumu geçişlerini ayıran kalıp
    r"^\[agent_node-\d+\]\s+(\d{2}:\d{2}:\d{2})\s+HA-(\d)\s+.*durum:\s+(\w+)\s+->\s+(\w+)"
)
BASLANGIC_SATIRI = re.compile(r"^KAYIT_BASLANGIC\s+(\d+)")  # kayıt başlangıcı kalıbı

AYRIM_SATIRI = re.compile(  # araçlar arası varış süresi kalıbı
    r"^HA-(\d) - HA-(\d): ([\d.]+) s \(20 s'den sapma ([+-][\d.]+) s\)")
SIRA_SATIRI = re.compile(r"^varış sırası HA-1/HA-2/HA-3: (\w+)")  # varış sırası kalıbı
KURAL_SATIRI = re.compile(  # hedef geçişi ve rota sapması kalıbı
    r"^HA-(\d): hedefe ([\d.]+) m \| max rota sapması ([\d.]+) m")
BEKLEME_SATIRI = re.compile(r"^toplam bekleme: yerde (\d+) s \| havada (\d+) s")  # bekleme kalıbı
KABUL_SATIRI = re.compile(r"^KABUL: (\w+)")  # kabul sonucu kalıbı

KART_SURE_S = 9.0  # kapanış kartının ekranda kalma süresi

GECIS_METNI = {  # görev geçişlerinde gösterilen altyazılar
    "MISSION_UPLOAD": "Araçların uçuş rotaları hazırlanıyor",
    "WAIT_TAKEOFF_SLOT": "HA-{v} planlanan kalkış zamanını bekliyor",
    "TAKEOFF": "HA-{v} kalkışa geçiyor",
    "CLIMB": "HA-{v} 400 m irtifaya tırmanıyor",
    "CRUISE": "HA-{v} hedefe doğru ilerliyor",
    "TERMINAL": "HA-{v} hedef bölgeye girdi",
    "ARRIVED": "HA-{v} hedefe vardı  {saat}",
    "RTL": "HA-{v} kalkış noktasına geri dönüyor (RTL)",
}

FILO_GECISLERI = {"MISSION_UPLOAD"}  # filo için bir kez gösterilen geçişler

ASS_BASLIK = """[Script Info]
ScriptType: v4.00+
PlayResX: 1920
PlayResY: 1080
WrapStyle: 2
ScaledBorderAndShadow: yes

[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, OutlineColour, BackColour, Bold, Italic, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Olay,DejaVu Sans,40,&H00FFFFFF,&H00000000,&H96000000,-1,0,3,3,0,2,60,60,52,1
Style: Varis,DejaVu Sans,46,&H0055FFAA,&H00000000,&H96000000,-1,0,3,3,0,2,60,60,52,1
Style: Hiz,DejaVu Sans,32,&H00FFD27A,&H00000000,&H96000000,-1,0,3,2,0,9,40,40,34,1
Style: Baslik,DejaVu Sans,54,&H00FFFFFF,&H00000000,&H96000000,-1,0,3,3,0,5,60,60,60,1
Style: Kapanis,DejaVu Sans Mono,38,&H00E8EAED,&H00000000,&H00000000,0,0,1,0,0,4,250,60,60,1

[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""  # ass altyazı başlığı ve stilleri


@dataclass
class Olay:
    """altyazıda gösterilecek tek bir görev olayı"""

    kaynak_s: float  # ham videodaki an
    metin: str  # ekranda görünecek yazı
    stil: str = "Olay"  # kullanılacak ass stili
    hedef_s: float = 0.0  # montajdan sonraki an


@dataclass
class Parca:
    """ham videodan tek hızda oynatılacak bir aralık"""

    baslangic: float  # ham videodaki başlangıç anı
    bitis: float  # ham videodaki bitiş anı
    hiz: float  # bu aralığın oynatma çarpanı


def _ass_zaman(saniye: float) -> str:
    """saniye değerini ass altyazı zaman biçimine çevirir"""
    saniye = max(saniye, 0.0)
    saat = int(saniye // 3600)
    dakika = int((saniye % 3600) // 60)
    kalan = saniye % 60
    return f"{saat:d}:{dakika:02d}:{kalan:05.2f}"


def olaylari_oku(log_yolu: Path) -> Tuple[List[Olay], Optional[str], float]:
    """kayıt günlüğünden olayları, sonucu ve simülasyon başlangıcını çıkarır"""
    satirlar = log_yolu.read_text(encoding="utf-8", errors="replace").splitlines()

    kayit_baslangic: Optional[int] = None
    for satir in satirlar:
        eslesme = BASLANGIC_SATIRI.match(satir)
        if eslesme:
            kayit_baslangic = int(eslesme.group(1))
            break
    if kayit_baslangic is None:
        raise SystemExit(
            "log dosyasinda KAYIT_BASLANGIC satiri yok; "
            "kayit scripts/record_video.sh ile alinmalidir")

    kayit_gunu = datetime.fromtimestamp(kayit_baslangic)
    olaylar: List[Olay] = []
    gorulen: set = set()
    simulasyon_baslangic_s: Optional[float] = None

    for satir in satirlar:
        eslesme = DURUM_SATIRI.match(satir)
        if not eslesme:
            continue
        saat_metni, arac, onceki, sonraki = eslesme.groups()
        saat = datetime.strptime(saat_metni, "%H:%M:%S").time()
        mutlak = datetime.combine(kayit_gunu.date(), saat)
        goreli = (mutlak - kayit_gunu).total_seconds()
        # gece yarısını aşan kayıt zamanını düzeltir
        if goreli < -3600.0:
            goreli += 86400.0
        if goreli < 0.0:
            continue

        if simulasyon_baslangic_s is None and onceki == "INIT" and sonraki == "CONNECTING":
            simulasyon_baslangic_s = goreli

        sablon = GECIS_METNI.get(sonraki)
        if sablon is None:
            continue

        # araç geçişlerini tek altyazıya indirir
        anahtar = (sonraki,) if sonraki in FILO_GECISLERI else (arac, sonraki)
        if anahtar in gorulen:
            continue
        gorulen.add(anahtar)

        olaylar.append(Olay(
            kaynak_s=goreli,
            metin=sablon.format(v=arac, saat=saat_metni),
            stil="Varis" if sonraki in {"TERMINAL", "ARRIVED"} else "Olay",
        ))

    olaylar.sort(key=lambda o: o.kaynak_s)

    # son varıştan sonra rtl bilgi kartı ekler
    varislar = [o for o in olaylar if "hedefe vardı" in o.metin]
    if varislar:
        olaylar.append(Olay(
            kaynak_s=varislar[-1].kaynak_s + RTL_ALTYAZI_GECIKMESI_S,
            metin="Üç araç da kalkış noktalarına geri dönüyor (RTL)",
        ))
        olaylar.sort(key=lambda o: o.kaynak_s)

    if simulasyon_baslangic_s is None:
        simulasyon_baslangic_s = olaylar[0].kaynak_s if olaylar else 0.0

    return olaylar, kapanis_karti_kur(satirlar), simulasyon_baslangic_s


def kapanis_karti_kur(satirlar: List[str]) -> Optional[str]:
    """analiz çıktısını kapanış kartına dönüştürür"""
    ayrimlar: List[Tuple[str, str, str]] = []
    yaklasmalar: List[str] = []
    sapmalar: List[float] = []
    sira = None
    havada = None
    kabul = None

    for satir in satirlar:
        temiz = satir.strip()
        e = AYRIM_SATIRI.match(temiz)
        if e:
            # çıkarma sırasını varış sırasına çevirir
            ayrimlar.append((e.group(2), e.group(1),
                             f"{e.group(3)} s  ({e.group(4)})"))
            continue
        e = SIRA_SATIRI.match(temiz)
        if e:
            sira = e.group(1)
            continue
        e = KURAL_SATIRI.match(temiz)
        if e:
            yaklasmalar.append(e.group(2))
            sapmalar.append(float(e.group(3)))
            continue
        e = BEKLEME_SATIRI.match(temiz)
        if e:
            havada = e.group(2)
            continue
        e = KABUL_SATIRI.match(temiz)
        if e:
            kabul = e.group(1)

    if kabul is None or not ayrimlar:
        return None

    # kart sütunlarını sabit genişlikte tutar
    etiket_g, deger_g = 18, 26

    def _satir(etiket: str, deger: str, not_: str = "") -> str:
        """kapanış kartı için sabit genişlikte bir satır kurar"""
        return f"{etiket:<{etiket_g}}{deger:<{deger_g}}{not_}"

    satirlar_kart = ["SONUÇ", ""]
    for i, (onceki, sonraki, deger) in enumerate(ayrimlar):
        satirlar_kart.append(
            _satir("Varış aralığı" if i == 0 else "", f"HA-{onceki} → HA-{sonraki} :  {deger}"))
    if sira:
        satirlar_kart.append(_satir(
            "Varış sırası",
            f"HA-1 / HA-2 / HA-3 {'doğru' if sira == 'DOĞRU' else 'YANLIŞ'}"))
    if yaklasmalar:
        satirlar_kart.append(_satir(
            "Hedefe yaklaşma", " / ".join(yaklasmalar) + " m", "(sınır 5 m)"))
    if sapmalar:
        satirlar_kart.append(_satir(
            "Rota sapması", f"en fazla {max(sapmalar):.0f} m", "(sınır 500 m)"))
    if havada is not None:
        satirlar_kart.append(_satir(
            "Havada bekleme", f"{havada} s"))
    sonuc_metni = (
        r"{\c&H0055FFAA&\b1}TEST BAŞARILI{\r}"
        if kabul == "GEÇTİ"
        else r"{\c&H004545FF&\b1}TEST BAŞARISIZ{\r}"
    )
    satirlar_kart += ["", sonuc_metni]
    return "\n".join(satirlar_kart)


def video_suresi(video: Path) -> float:
    """ffprobe ile videonun toplam süresini saniye olarak okur"""
    cikti = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration",
         "-of", "default=noprint_wrappers=1:nokey=1", str(video)],
        capture_output=True, text=True, check=True)
    return float(cikti.stdout.strip())


def parcalari_kur(olaylar: List[Olay], toplam_s: float,
                  baslik_kaynak_s: float) -> List[Parca]:
    """videoyu normal ve hızlı oynatılacak parçalara böler"""
    baslik_kaynak_s = min(max(baslik_kaynak_s, 0.0), toplam_s)
    pencereler: List[Tuple[float, float]] = [
        (baslik_kaynak_s, min(baslik_kaynak_s + BASLIK_SURE_S, toplam_s))
    ]
    for olay in olaylar:
        sure = BASARI_SURE_S if olay.stil == "Varis" else OLAY_SURE_S
        pencereler.append((
            max(olay.kaynak_s, 0.0),
            min(olay.kaynak_s + sure, toplam_s),
        ))

    # kesişen olay pencerelerini birleştirir
    pencereler.sort()
    birlesik: List[List[float]] = []
    for bas, bit in pencereler:
        if birlesik and bas <= birlesik[-1][1]:
            birlesik[-1][1] = max(birlesik[-1][1], bit)
        else:
            birlesik.append([bas, bit])

    parcalar: List[Parca] = []
    imlec = 0.0
    for bas, bit in birlesik:
        if bas > imlec:
            parcalar.append(Parca(imlec, bas, SEYIR_HIZI))
        parcalar.append(Parca(bas, bit, 1.0))
        imlec = bit
    if imlec < toplam_s:
        parcalar.append(Parca(imlec, toplam_s, SEYIR_HIZI))
    return [p for p in parcalar if p.bitis - p.baslangic > 0.05]


def zamanlari_esle(olaylar: List[Olay], parcalar: List[Parca],
                   baslik_kaynak_s: float) -> Tuple[float, float]:
    """kaynak anlarını montaj zaman eksenine taşır"""
    def hedefe(kaynak_s: float) -> float:
        """ham videodaki bir anı montaj zamanına çevirir"""
        gecen = 0.0
        for p in parcalar:
            if kaynak_s >= p.bitis:
                gecen += (p.bitis - p.baslangic) / p.hiz
            elif kaynak_s > p.baslangic:
                gecen += (kaynak_s - p.baslangic) / p.hiz
                return gecen
            else:
                return gecen
        return gecen

    for olay in olaylar:
        olay.hedef_s = hedefe(olay.kaynak_s)
    toplam_s = sum((p.bitis - p.baslangic) / p.hiz for p in parcalar)
    return toplam_s, hedefe(baslik_kaynak_s)


def ass_yaz(olaylar: List[Olay], parcalar: List[Parca], toplam_s: float,
            baslik_hedef_s: float, sonuc: Optional[str], yol: Path) -> None:
    """olayları ve kapanış kartını ass altyazı dosyasına yazar"""
    satirlar = [ASS_BASLIK]

    def ekle(bas: float, bit: float, stil: str, metin: str) -> None:
        """altyazı listesine biçimlendirilmiş bir ass satırı ekler"""
        # ass içinde satır başı boşluklarını korur
        korunmus = []
        for satir in metin.split("\n"):
            girinti = len(satir) - len(satir.lstrip(" "))
            korunmus.append("\\h" * girinti + satir[girinti:])
        temiz = "\\N".join(korunmus)
        satirlar.append(
            f"Dialogue: 0,{_ass_zaman(bas)},{_ass_zaman(bit)},{stil},,0,0,0,,{temiz}")

    ekle(baslik_hedef_s, baslik_hedef_s + BASLIK_SURE_S, "Baslik",
         "OASY üç araçlı varış görevi\\N"
         "HA-1 → HA-2 → HA-3, aralarında 20 saniye")

    # yakın olayları tek altyazı bloğunda toplar
    gruplar: List[List[Olay]] = []
    for olay in olaylar:
        if gruplar and olay.hedef_s - gruplar[-1][0].hedef_s <= GRUPLAMA_PENCERESI_S:
            gruplar[-1].append(olay)
        else:
            gruplar.append([olay])

    for i, grup in enumerate(gruplar):
        bas = grup[0].hedef_s
        stil = "Varis" if any(o.stil == "Varis" for o in grup) else "Olay"
        sure = BASARI_SURE_S if stil == "Varis" else OLAY_SURE_S
        # altyazıyı sonraki gruptan önce kaldırır
        if i + 1 < len(gruplar):
            sure = min(sure, max(gruplar[i + 1][0].hedef_s - bas - 0.2, 1.2))
        ekle(bas, bas + sure, stil, "\n".join(o.metin for o in grup))

    # hızlı bölümlerde hız çarpanını gösterir
    imlec = 0.0
    for p in parcalar:
        cikti_sure = (p.bitis - p.baslangic) / p.hiz
        if p.hiz > 1.0:
            ekle(imlec, imlec + cikti_sure, "Hiz", f"▶ {p.hiz:.0f}× hızlandırıldı")
        imlec += cikti_sure

    # kapanış kartını videonun sonuna ekler
    if sonuc:
        ekle(toplam_s + 0.15, toplam_s + KART_SURE_S, "Kapanis", sonuc)

    yol.write_text("\n".join(satirlar) + "\n", encoding="utf-8")


def ffmpeg_calistir(ham: Path, parcalar: List[Parca], ass: Path, cikti: Path,
                    kart_var: bool) -> None:
    """video parçalarını birleştirip altyazılı çıktıyı üretir"""
    zincirler = []
    etiketler = []
    for i, p in enumerate(parcalar):
        etiket = f"v{i}"
        zincirler.append(
            f"[0:v]trim=start={p.baslangic:.3f}:end={p.bitis:.3f},"
            f"setpts=(PTS-STARTPTS)/{p.hiz:.4f}[{etiket}]")
        etiketler.append(f"[{etiket}]")
    zincirler.append(f"{''.join(etiketler)}concat=n={len(parcalar)}:v=1:a=0[birlesik]")
    # görüntüyü altyazıdan önce hedef çözünürlüğe getirir
    zincirler.append(
        f"[birlesik]scale={HEDEF_G}:-2,"
        f"pad={HEDEF_G}:{HEDEF_Y}:(ow-iw)/2:(oh-ih)/2:color=black,"
        f"setsar=1,format=yuv420p[olcekli]")

    girdiler = ["-i", str(ham)]
    kaynak = "[olcekli]"
    if kart_var:
        # kapanış kartı için düz zemin ekler
        girdiler += ["-f", "lavfi", "-i",
                     f"color=c=0x111417:s={HEDEF_G}x{HEDEF_Y}:d={KART_SURE_S}:r=15"]
        zincirler.append("[1:v]setsar=1,format=yuv420p[kart]")
        zincirler.append("[olcekli][kart]concat=n=2:v=1:a=0[tamam]")
        kaynak = "[tamam]"

    kacisli = str(ass).replace("\\", "/").replace(":", r"\:").replace("'", r"\'")
    zincirler.append(f"{kaynak}subtitles='{kacisli}'[cikti]")

    komut = [
        "ffmpeg", "-loglevel", "error", "-stats", "-y",
        *girdiler,
        "-filter_complex", ";".join(zincirler),
        "-map", "[cikti]",
        "-c:v", "libx264", "-preset", "medium", "-crf", "21",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(cikti),
    ]
    print("ffmpeg calistiriliyor…")
    subprocess.run(komut, check=True)


def main() -> int:
    """video düzenleme girdilerini okuyup montaj işlemini yürütür"""
    ayristirici = argparse.ArgumentParser(description=__doc__)
    ayristirici.add_argument("ham_video", type=Path)
    ayristirici.add_argument("olay_log", type=Path)
    ayristirici.add_argument("-o", "--cikti", type=Path, default=None)
    ayristirici.add_argument("--kuru", action="store_true",
                             help="ffmpeg calistirma, plani yazdir")
    args = ayristirici.parse_args()

    for yol in (args.ham_video, args.olay_log):
        if not yol.exists():
            print(f"bulunamadi: {yol}", file=sys.stderr)
            return 1

    cikti = args.cikti or args.ham_video.with_name(
        args.ham_video.name.replace("ham_", "oasy_", 1))
    ass_yolu = cikti.with_suffix(".ass")

    olaylar, sonuc, baslik_kaynak_s = olaylari_oku(args.olay_log)
    if not olaylar:
        print("logda altyaziya uygun olay bulunamadi", file=sys.stderr)
        return 1

    ham_sure = video_suresi(args.ham_video)
    parcalar = parcalari_kur(olaylar, ham_sure, baslik_kaynak_s)
    yeni_sure, baslik_hedef_s = zamanlari_esle(
        olaylar, parcalar, baslik_kaynak_s)
    ass_yaz(olaylar, parcalar, yeni_sure, baslik_hedef_s, sonuc, ass_yolu)

    print(f"ham sure   : {ham_sure / 60.0:.1f} dk")
    print(f"montaj     : {yeni_sure / 60.0:.1f} dk  ({len(parcalar)} parca)")
    print(f"olay sayisi: {len(olaylar)}")
    print(f"altyazi    : {ass_yolu}")
    for olay in olaylar:
        print(f"  {_ass_zaman(olay.hedef_s)}  {olay.metin}")

    if args.kuru:
        return 0

    ffmpeg_calistir(args.ham_video, parcalar, ass_yolu, cikti, sonuc is not None)
    print(f"\nvideo hazir: {cikti}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
