# 3. Varış Zamanı Kontrolü, Rüzgâr Kompanzasyonu ve Algoritmalar

> Vaka belgesi, teknik raporda **"Varış zamanı kontrolü, rüzgar kompanzasyonu ve
> kullandıysanız özel manevraların/algoritmaların mantığını anlatan algoritma
> açıklamaları"** başlığını istiyor. Bu bölüm o başlığın özetidir; her algoritma
> kendi kavram sayfasında ayrıntılı olarak ele alınmıştır.

## 3.1 Temel Fikir

Sistem tek bir soruyu sürekli cevaplar:

> **"Şu an gittiğim hızla hedefe ne zaman varırım, ve bu, varmam gereken andan
> ne kadar farklı?"**

Bu farkı kapatmak için elde dört araç var ve **yetkileri giderek daralır**:

| Araç | Yetki | Nerede | Kavram |
|---|---|---|---|
| Kalkış gecikmesi | sınırsız, bedava | yerde | [05](kavramlar/05-kalkis-slotu-ve-yer-gecikmesi.md) |
| Plan revizyonu | sınırsız, bedava | kalkış civarı | [04](kavramlar/04-plan-revizyonu.md) |
| Kapı loiteri | sınırsız, **tek seferlik** | 2.5 km öncesi | [12](kavramlar/12-son-yasal-kapi.md) |
| Hız kontrolü | ±%20 civarı | her yerde | [11](kavramlar/11-varis-zamani-kontrolcusu.md) |

Belgenin madde 5'inde sayılan dört yöntemden **üçünü** kullanıyoruz. Dördüncüsü
(S-manevrası) uygulandı, ölçüldü ve kaldırıldı —
[04 - Geliştirme ve Testler](04-gelistirme-ve-testler.md#s-manevrası-neden-yok).

## 3.2 Varış Zamanı Kontrolü

### Hata sinyali

$$e = \text{ETA} - t_{\text{kalan}}$$

Pozitif = geç kalacağım. Kontrolcü gerekli hızı oranla bulur:

$$v_{\text{gerekli}} = v_{\text{komut}} \times \frac{\text{ETA}}{t_{\text{kalan}}}$$

Sonra dört koruma katmanından geçirir:

```
hata ──► olu bant (0.5 s) ──► oran ──► doygunluk [13,28] ──► rate limit ──► gonderme esigi
```

Ayrıntı: [11 - Varış Zamanı Kontrolcüsü](kavramlar/11-varis-zamani-kontrolcusu.md)

### En pahalı parametre

`airspeed_rate_limit_mps2` başlangıçta **0.5 m/s²** idi. Ölçüldü:

| Değer | `rate_limited=True` olan tick |
|---|---|
| 0.5 m/s² | **865 / 878 = %98.5** |
| 1.5 m/s² | 350 / 568 = %62 |

%98.5'te kontrolcü uçuşun neredeyse tamamında istediğini uygulayamıyordu —
kapalı çevrim değil **açık çevrim rampa** gibi çalışıyordu. 28'den 13'e inmek
0.5 m/s²'de 30 saniye ve ~615 metre sürüyordu.

Bu, aylarca "araç düzeltmeyi yetiştiremiyor" diye gözlemlenen davranışın
altındaki mekanizmaydı.

## 3.3 Rüzgâr Kompanzasyonu

### Üç katman

**1. Kestirim.** Rüzgâr, yer hızı ile hava hızının vektör farkıdır:

$$\vec{v}_{\text{rüzgâr}} = \vec{v}_{\text{yer}} - R(q)\,\vec{v}_{\text{hava}}^{\,FLU}$$

Kritik olgu: ArduPilot'un yayınladığı hava hızı vektörü EKF'in rüzgâr durumunu
içerir, dolayısıyla bu hesap cebirsel olarak **EKF'in kendi kestirimini geri
okur**. Sonuç: kestirim kalitesi bizim filtremize değil, `ARSPD_USE` ve
`EK3_WIND_P_NSE` parametrelerine bağlıdır.

Ayrıntı: [06 - Rüzgâr Kestirimi](kavramlar/06-ruzgar-kestirimi.md)

**2. Rota süresi modeli.** Rüzgâr bacak bacak ayrıştırılır:

$$v_{\text{yer}} = \sqrt{V_a^2 - w_{\perp}^2} + w_{\parallel}$$

Yan rüzgâr terimi ($w_\perp$) çoğu kişinin atladığı yerdir: uçak yengeç açısı
yaparak yan rüzgârı dengeler ve bu, ileri hızından çalar.

**Aynı rüzgâr, HA-3'ün dört bacağında bambaşka etki** (9.4 m/s @ 330°, hava hızı
16.8 m/s):

| Bacak | Rota | $w_\parallel$ | $w_\perp$ | Yer hızı |
|---|---|---|---|---|
| WP1→WP2 | 22° | −5.73 | −7.45 | **9.32** |
| WP2→WP3 | 339° | −9.29 | −1.44 | **7.45** |
| WP3→WP4 | 294° | −7.60 | +5.53 | **8.26** |
| WP4→hedef | 135° | +9.08 | −2.43 | **25.71** |

3.4 kat fark. Bu yüzden rüzgâr düzeltmesi tek bir katsayı olamaz.

Ayrıntı: [07 - Rüzgâr Düzeltmeli Rota Süresi](kavramlar/07-ruzgar-duzeltmeli-rota-suresi.md)

**3. Belirsizlik zarfı.** Rüzgâr değişir; plan bunu hesaba katmalı. E ve L
sınırları rüzgâr zarfı taranarak bulunur.

Ayrıntı: [14 - Robust E/L Sınırları](kavramlar/14-robust-e-l-sinirlari.md)

### En pahalı hata

Önceki sürüm ETA'yı **ölçülen yer hızına bölerek** hesaplıyordu. Ölçülen hız
dönüşlerde çöker; ETA ±15 saniye salınıyordu.

Model tabanlı hesaba geçince **salınım ±0.7 s'ye indi** — ve bu tek düzeltme üç
ayrı belirtiyi birden kapattı.

Ders kod yorumlarında kayıtlı: *ölçülen hıza bölmeyin, modeli kullanın.*

## 3.4 Merkeziyetsiz Zamanlama

Ortak çıpa, komut olmadan kurulur:

$$A = \max_i \left( E_i - 20(i-1) \right), \qquad T_i = A + 20(i-1)$$

`max` sırasız olduğu için aynı veriyi gören üç araç aynı sonucu bulur.

**Doğrulanmış örnek** — hesap ölçümü yarım saniye içinde öngörüyor:

| Araç | Hesaplanan yer beklemesi | Ölçülen | Fark |
|---|---|---|---|
| HA-1 | 0 s | 0.0 s | — |
| HA-2 | 15 s | 14.5 s | 0.5 s |
| HA-3 | 168 s | 168.1 s | **0.1 s** |

Ayrıntı: [02 - Merkeziyetsiz Çıpa](kavramlar/02-merkeziyetsiz-capa.md)

## 3.5 Terminal Faz: 2 km Kuralının Getirdiği Zorluk

Belge madde 6: hedefin 2 km çemberi içinde loiter **kesinlikle yasak**.

Ama geometri şunu söylüyor: HA-3 o çembere girdiğinde rota olarak **hâlâ 4307
metre** kalıyor — rota ilmek atıyor. Ölçülen terminal faz süresi **214 saniye**,
görevin %36'sı.

Yani görevin üçte birinde tek yetki hız. Ve kuyruk rüzgârında hız yetkisi sıfıra
inebiliyor:

| | Değer |
|---|---|
| Asgari hava hızı | 13.0 m/s |
| Kuyruk rüzgârı bileşeni | +9.08 m/s |
| **Elde edilebilen en düşük yer hızı** | **21.9 m/s** |
| Planın varsaydığı nominal yer hızı | 22.9 m/s |

**Gaz tamamen kesilse bile araç nominal hızda gidiyor.** Bu, sistemin en zorlu
kısıtıdır.

İki mekanizma bunun için var:

**Son yasal kapı** — erkenliği yasak bölgeye girmeden soğurur. Kapı, rotanın
2500 m çemberine son girişidir; bırakma anı $[T-L, T-E]$ penceresinden
türetilir.

Ayrıntı: [12 - Son Yasal Kapı](kavramlar/12-son-yasal-kapi.md)

**Terminal rezerv** — hız yetkisinin her iki ucuna kalan payı sürekli ölçer,
biri tükenmeye başlayınca karşı yöne baskı yapar. Amaç: araç hız tabanına
yapışmasın, düzeltme yetkisini yedekte tutsun.

Ölçüldü: başarılı koşuda araç son yaklaşmada 16.8 m/s komut ediyordu — tabanda
değil, 3.8 m/s yavaşlama payı elinde.

Ayrıntı: [13 - Terminal Rezerv](kavramlar/13-terminal-rezerv.md)

## 3.6 Varış Ölçümü

Başarı ölçütü 5 m çemberine giriş anıdır. Telemetri 20 Hz geldiği için araç
çemberi teğet geçerse örnekler onu kaçırabilir:

| En yakın geçiş | Çemberde kalan kiriş | İçeride kalan örnek |
|---|---|---|
| 4.79 m | 2.87 m | ~2 |
| 4.99 m | 0.63 m | **0–1** |

Çözüm: ardışık iki konum arasındaki doğru parçası çemberle kesiştirilip giriş
anı **interpolasyonla** bulunur.

Ayrıntı: [10 - Varış Tespiti](kavramlar/10-varis-tespiti.md)

## 3.7 Algoritma Haritası

Hangi soru hangi sayfada:

| Soru | Kavram sayfası |
|---|---|
| Görev akışı nasıl yönetiliyor? | [01 - Görev Durum Makinesi](kavramlar/01-gorev-durum-makinesi.md) |
| Ortak zaman referansı komutsuz nasıl kurulur? | [02 - Merkeziyetsiz Çıpa](kavramlar/02-merkeziyetsiz-capa.md) |
| Hangi peer'a ne kadar güvenilir? | [03 - Peer Yönetimi ve Tazelik](kavramlar/03-peer-yonetimi-ve-tazelik.md) |
| Rüzgâr öğrenilince plan ne olur? | [04 - Plan Revizyonu](kavramlar/04-plan-revizyonu.md) |
| Kalkış ne zaman yapılmalı? | [05 - Kalkış Slotu](kavramlar/05-kalkis-slotu-ve-yer-gecikmesi.md) |
| Rüzgâr nasıl ölçülüyor? | [06 - Rüzgâr Kestirimi](kavramlar/06-ruzgar-kestirimi.md) |
| Rota kaç saniye sürer? | [07 - Rüzgâr Düzeltmeli Rota Süresi](kavramlar/07-ruzgar-duzeltmeli-rota-suresi.md) |
| Rotanın neresindeyim? | [08 - ETA ve Kalan Mesafe](kavramlar/08-eta-ve-kalan-mesafe.md) |
| Mesafe ve çember hesapları | [09 - Jeodezi](kavramlar/09-jeodezi.md) |
| Varış anı nasıl belirlenir? | [10 - Varış Tespiti](kavramlar/10-varis-tespiti.md) |
| Hız nasıl ayarlanıyor? | [11 - Varış Zamanı Kontrolcüsü](kavramlar/11-varis-zamani-kontrolcusu.md) |
| Son bekleme fırsatı nerede? | [12 - Son Yasal Kapı](kavramlar/12-son-yasal-kapi.md) |
| Terminal fazda yetki nasıl korunur? | [13 - Terminal Rezerv](kavramlar/13-terminal-rezerv.md) |
| Ulaşılabilirlik nasıl sınırlanıyor? | [14 - Robust E/L Sınırları](kavramlar/14-robust-e-l-sinirlari.md) |
| Haberleşme nasıl ayrıştırılıyor? | [15 - DDS Mimarisi](kavramlar/15-dds-mimarisi-ve-domain-ayrimi.md) |
| Test rüzgârı gerçekçi mi? | [16 - Rüzgâr Profili ve Gerçekçilik](kavramlar/16-ruzgar-profili-ve-gercekcilik.md) |

## 3.8 Kullanılmayan Yöntem: S-Manevrası

Madde 5 dört yöntem sayıyor; biz üçünü kullanıyoruz. Dördüncüsü — uçuş yolu
uzatma — **uygulandı, uçuşta doğrulandı ve kaldırıldı.**

Kısa özet:

- **Yürütme çözüldü.** GUIDED modu yol takibi yapmıyor (`ModeGuided::navigate`
  → `update_loiter`, `set_guided_WP` crosstrack'i kapatıyor). Alternatif olarak
  AUTO görev yuvalarına uçuş sırasında kısmi yazma geliştirildi ve **uçuşta
  doğrulandı** (151 m sapma ölçüldü, planlanan 138 m).
- **Geometri uçurulamadı.** Manevra asgari hızda tetiklendiği için dar
  dönüşlerde araç yol kaybetti: yer hızı 3.5 m/s'ye düştü, kalan mesafe **arttı**.
- **Entegrasyon planlama katmanını kararsızlaştırdı.** Araç hedefe 14 m
  mesafedeyken model "rota kalan 4971 m" gördü, mandallı çıpa 4474 saniyeye
  tırmandı, görev bitmedi.

Belge S-manevrasını **zorunlu tutmuyor** (madde 5 "kullanabilir", madde 6
"kullanılabilirsiniz" kipinde). Üç yöntemle 20 s şartı üç senaryoda da
sağlandığı için kaldırıldı.

Ayrıntılı ölçümler ve karar gerekçesi:
[04 - Geliştirme ve Testler](04-gelistirme-ve-testler.md#s-manevrası-neden-yok)
