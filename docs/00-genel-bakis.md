# OASY Teknik Rapor: Genel Bakış

Üç sabit kanatlı İHA'nın (ArduPlane 4.6.3 SITL) farklı pistlerden otonom kalkıp
ortak bir hedefe **HA-1 → HA-2 → HA-3** sırasıyla ve aralarında **tam 20 saniye**
farkla varmasını sağlayan **merkeziyetsiz** sistem.

Merkezi bir yer kontrol istasyonu ya da master node yoktur; her araç kendi
kararını diğerlerinin yayınlarını dinleyerek bağımsız verir.

## Sonuç Özeti

Mevcut derlemeyle, senaryo başına iki koşu, **altısı da geçti**:

| Senaryo | Koşu 1 | Koşu 2 | Havada bekleme |
|---|---|---|---|
| Sakin | −0.00 / −0.00 s | −0.00 / −0.01 s | 0 s |
| Sabit 8 m/s | −0.20 / +0.02 s | −0.29 / +0.09 s | 62 s |
| Değişken rüzgâr | −0.13 / +0.10 s | −0.11 / +0.17 s | 0 s |

| Ölçüt | Sınır | En kötü gözlenen |
|---|---|---|
| Ardışık varış farkı | 20 s ± 1.0 s | **+0.17 s** |
| Varış sırası | HA-1/2/3 | doğru |
| Hedefe yaklaşma | ≤ 5 m | 4.99 m |
| Rota sapması | ≤ 500 m | 127 m |

Beklemenin neredeyse tamamı **yerde** yapılıyor (madde 8: "bekleme süreleri en
az olacak şekilde optimal senaryo").

## Belge Şartları ve Karşılıkları

| Madde | Şart | Karşılık |
|---|---|---|
| 1 | Merkeziyetsiz, 20 s fark | [Merkeziyetsiz Çıpa](kavramlar/02-merkeziyetsiz-capa.md) |
| 1 | Otomatik kalkış, GCS'siz | [Görev Durum Makinesi](kavramlar/01-gorev-durum-makinesi.md) |
| 2 | Sıra HA-1/2/3, 5 m kabul, RTL | [Varış Tespiti](kavramlar/10-varis-tespiti.md) |
| 3 | 400 m MSL | `cruise_alt_msl_m`, görev üretimi |
| 4 | WP yarıçapı ≤400 m, sapma ≤500 m | 120 m seçildi; [Jeodezi](kavramlar/09-jeodezi.md) |
| 5 | Zamanlama yöntemleri | 4 yöntemden **3'ü** [Algoritmalar](03-algoritmalar.md) |
| 6 | 2 km içinde loiter yasak | [Son Yasal Kapı](kavramlar/12-son-yasal-kapi.md) |
| 7 | Değişken rüzgâr, robustluk | [Rüzgâr Profili](kavramlar/16-ruzgar-profili-ve-gercekcilik.md) |
| 8 | Bekleme asgari | [Kalkış Slotu](kavramlar/05-kalkis-slotu-ve-yer-gecikmesi.md) |
| yok | AP_DDS ile telemetri | [DDS Mimarisi](kavramlar/15-dds-mimarisi-ve-domain-ayrimi.md) |

## Rapor Yapısı

Belgenin istediği dört başlık:

| # | Bölüm | İçerik |
|---|---|---|
| 1 | [Sistem Mimarisi](01-sistem-mimarisi.md) | Genel mimari ve merkeziyetsiz kontrol yaklaşımı |
| 2 | [Haberleşme Akışı](02-haberlesme-akisi.md) | Node'lar arası akış, paket yapısı, akış şemaları |
| 3 | [Algoritmalar](03-algoritmalar.md) | Varış kontrolü, rüzgâr kompanzasyonu, özel algoritmalar |
| 4 | [Geliştirme ve Testler](04-gelistirme-ve-testler.md) | Süreç, test adımları, problemler, sonuçlar |

Her mekanizma ayrıca **kendi kavram sayfasında** ayrıntılı olarak ele alınmıştır:
neden var, nasıl çalışır, matematiği, **gerçek uçuş verisiyle çalışılmış örneği**
ve sınırlamaları.

## Kavram Sayfaları

### Koordinasyon

| Sayfa | Çekirdek fikir |
|---|---|
| [01 - Görev Durum Makinesi](kavramlar/01-gorev-durum-makinesi.md) | 14 durum, tek yönlü akış, 20 Hz döngü |
| [02 - Merkeziyetsiz Çıpa](kavramlar/02-merkeziyetsiz-capa.md) | $A = \max_i(E_i - 20(i-1))$ komutsuz ortak referans |
| [03 - Peer Yönetimi ve Tazelik](kavramlar/03-peer-yonetimi-ve-tazelik.md) | TAZE / ESKİ / KAYIP; deterministik kaynak seçimi |
| [04 - Plan Revizyonu](kavramlar/04-plan-revizyonu.md) | Rüzgâr öğrenilince plan **yalnızca ileri** çekilir |
| [05 - Kalkış Slotu](kavramlar/05-kalkis-slotu-ve-yer-gecikmesi.md) | Yerde beklemek bedava; hesap ölçümü 0.1 s içinde öngörüyor |

### Kestirim

| Sayfa | Çekirdek fikir |
|---|---|
| [06 - Rüzgâr Kestirimi](kavramlar/06-ruzgar-kestirimi.md) | Hesap cebirsel olarak **EKF'in kendi durumunu geri okuyor** |
| [07 - Rüzgâr Düzeltmeli Rota Süresi](kavramlar/07-ruzgar-duzeltmeli-rota-suresi.md) | Aynı rüzgâr dört bacakta 7.45-25.71 m/s |
| [08 - ETA ve Kalan Mesafe](kavramlar/08-eta-ve-kalan-mesafe.md) | İzdüşüm testi; ölçülen hıza bölmemek |
| [09 - Jeodezi](kavramlar/09-jeodezi.md) | Ne zaman elipsoit, ne zaman düzlem |
| [10 - Varış Tespiti](kavramlar/10-varis-tespiti.md) | Teğet geçişte marj **tek örnek** |

### Kontrol

| Sayfa | Çekirdek fikir |
|---|---|
| [11 - Varış Zamanı Kontrolcüsü](kavramlar/11-varis-zamani-kontrolcusu.md) | Dört koruma katmanı; rate limit %98.5 doygundu |
| [12 - Son Yasal Kapı](kavramlar/12-son-yasal-kapi.md) | 2 km yasağından önceki son bekleme fırsatı |
| [13 - Terminal Rezerv](kavramlar/13-terminal-rezerv.md) | Hız yetkisini yedekte tutmak |
| [14 - Robust E/L Sınırları](kavramlar/14-robust-e-l-sinirlari.md) | Ulaşılabilirlik zarfı; bozucu > kontrol yetkisi |

### Altyapı ve Doğrulama

| Sayfa | Çekirdek fikir |
|---|---|
| [15 - DDS Mimarisi](kavramlar/15-dds-mimarisi-ve-domain-ayrimi.md) | Tek süreçte iki domain; sessiz arıza tespiti |
| [16 - Rüzgâr Profili ve Gerçekçilik](kavramlar/16-ruzgar-profili-ve-gercekcilik.md) | Kendi testimiz fırtına cephesi seviyesindeymiş |

## Öne Çıkan Üç Bulgu

### 1. Ölçülen hıza bölmek zamanlamayı bozuyor

ETA `kalan_mesafe / ölçülen_yer_hızı` ile hesaplanıyordu. Dönüşlerde ilerleme
hızı çöküyor ve ETA fırlıyordu, aynı yerde, aynı hızda **30 saniyelik hayalî
gecikme**.

Model tabanlı hesaba geçince salınım **±15 s → ±0.7 s**. Tek düzeltme üç ayrı
belirtiyi birden kapattı.

### 2. Son bacakta yavaşlama yetkisi sıfıra inebiliyor

HA-3'ün son bacağı 135° rotada. 330°'den gelen 9.4 m/s rüzgâr bu bacakta
**+9.08 m/s** kuyruk bileşeni veriyor:

| | Değer |
|---|---|
| Asgari hava hızı | 13.0 m/s |
| Elde edilebilen en düşük yer hızı | **21.9 m/s** |
| Planın varsaydığı nominal yer hızı | 22.9 m/s |

Gaz tamamen kesilse bile araç nominal hızda gidiyor. [Son Yasal Kapı](kavramlar/12-son-yasal-kapi.md)
ve [Terminal Rezerv](kavramlar/13-terminal-rezerv.md) tam bunun için var.

### 3. Kendi doğrulama testimiz gerçek dışıydı

Profilimiz rüzgârı 20 saniyede 110° döndürüyordu, **5.5 °/s**, yani fırtına
çıkış cephesi seviyesi, 12 dakikada dört kez.

Bu yüzden üç ayrı "çözüm" boşa gitti. Profil cephe geçişi seviyesine
(**0.50 °/s**) indirilince, **kod değişmeden** üç senaryo da geçti.

Test zayıflamadı. Rüzgâr hızları aynı kaldı ve yeni profil son
bacakta kuyruk rüzgârını **daha uzun süre** üretiyor (5 adımın 3'ü, eskiden 1'i).

> Bir doğrulama senaryosu başarısızlık üretiyorsa, önce senaryonun
> kendisinin fiziksel olarak savunulabilir olduğunu doğrula.

## Bilinen Sınırlamalar

Dürüstlük gereği açıkça listelenir:

| Eksik | Durum |
|---|---|
| `FAILSAFE` işleyicisi | `safety_manager.py` **boş** (0 satır) |
| S-manevrası | Uygulandı, ölçüldü, **kaldırıldı** [gerekçe](04-gelistirme-ve-testler.md#44-s-manevrası-neden-yok) |
| Uç durum rüzgârı | 5.5 °/s dönüşte sapma −2.55 s (ölçülmüş sınır) |
| Öncüye bağımlılık | Rüzgâr bilgisi tek kaynaktan |
| Split-brain | Ağ bölünmesine karşı koruma yok |
| Tek makine | `monotonic_ns` süreç yerel |

## Kaynak Kod Haritası

| Dizin | İçerik |
|---|---|
| `ros2_ws/src/oasy_uav_agent/` | Agent paketi durum makinesi, kestirim, kontrol, koordinasyon |
| `ros2_ws/src/oasy_interfaces/` | `VehicleStatus.msg` colcon'a hazır |
| `ros2_ws/src/oasy_bringup/` | Launch dosyaları, araç konfigürasyonları, SITL parametreleri |
| `scripts/` | Başlatma, rüzgâr profili, koşu analizi |
| `tests/unit/` | 161 birim testi |
| `ardupilot/` | ArduPlane 4.6.3 (klonlandı, **değişiklik yapılmadı**) |

Ayrıntılı dosya haritası ve çalıştırma talimatları `README.md`'de.

## Doğrulamayı Tekrarlamak

```bash
# birim testler
python3 -m pytest tests/unit -q

# senaryo koşusu (sakin)
scripts/start_all.sh 1

# degisken ruzgar
scripts/start_all.sh 1 ros2_ws/src/oasy_bringup/params/wind/variable.parm
python3 scripts/wind_profile.py

# ruzgar profilini yazdir (rapor icin)
python3 scripts/wind_profile.py --print-profile
python3 scripts/wind_profile.py --print-profile --extreme

# sonuc analizi
python3 scripts/analyze_run.py --log logs/run_YYYYMMDD_HHMMSS/agents.log
```
