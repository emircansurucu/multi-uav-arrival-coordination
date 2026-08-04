# 4. Geliştirme Süreci, Test Adımları, Problemler ve Sonuçlar

> Vaka belgesi, teknik raporda **"Geliştirme süreci, test adımları, karşılaşılan
> problemler ve test sonuçları"** başlığını istiyor. Bu bölüm o başlığın
> karşılığıdır ve bilinçli olarak **başarısızlıkları da içerir** — bir sistemin
> nasıl çalıştığını anlamanın en hızlı yolu, neyin çalışmadığını görmektir.

## 4.1 Doğrulama Yöntemi

### Kabul ölçütleri

`scripts/analyze_run.py` her koşuyu dört ölçütle denetler:

| Ölçüt | Kaynak | Sınır |
|---|---|---|
| Varış sırası | Madde 2 | HA-1 → HA-2 → HA-3 |
| Ardışık varış farkı | Madde 1 | 20 s ± 1.0 s |
| Hedefe yaklaşma | Madde 2 | ≤ 5 m |
| Rota sapması | Madde 4 | ≤ 500 m |

Araç, koordinasyon domainini dinleyip varış anlarını doğrudan yayınlardan okur;
ayrıca agent loglarını ayrıştırıp bekleme sürelerini ve sapmaları çıkarır.

### Senaryolar

| Senaryo | Rüzgâr | Amaç |
|---|---|---|
| **Sakin** | yok | Temel doğruluk |
| **Sabit** | 8 m/s, sabit yön | Sürekli bozucu |
| **Değişken** | 4–10 m/s, 0.50 °/s dönüş | Madde 7 |
| *Uç durum* | 4–10 m/s, **5.5 °/s** dönüş | Algoritmanın sınırı |

Uç durum varsayılan değildir; `--extreme` bayrağıyla çalıştırılır ve raporda
sınır göstergesi olarak kullanılır ([16](kavramlar/16-ruzgar-profili-ve-gercekcilik.md)).

### Birim testler

201 birim testi, uçuş gerektirmeyen mantığı kapsar: çıpa hesabı, tazelik
sınıflaması, jeodezi, varış tespiti, kontrolcü sınırlayıcıları, kapı penceresi,
görev üretimi.

```bash
python3 -m pytest tests/unit -q
```

## 4.2 Test Sonuçları

Mevcut derlemeyle, senaryo başına iki koşu — **altısı da geçti**:

| Senaryo | Koşu 1 | Koşu 2 | Havada bekleme |
|---|---|---|---|
| Sakin | −0.00 / −0.00 s | −0.00 / −0.01 s | 0 s |
| Sabit 8 m/s | −0.20 / +0.02 s | −0.29 / +0.09 s | 62 s |
| Değişken | −0.13 / +0.10 s | −0.11 / +0.17 s | 0 s |

Diğer ölçütler tüm koşularda sağlandı:

| Ölçüt | En kötü gözlenen | Sınır |
|---|---|---|
| Hedefe yaklaşma | 4.99 m | 5 m |
| Rota sapması | 127 m | 500 m |
| Varış sırası | doğru | — |

**Zorlu koşulun gerçekten yaşandığının doğrulanması.** Değişken rüzgâr
koşularında HA-3'ün son bacağında ölçülen rüzgâr:

| Koşu | Rüzgâr | Yer hızı |
|---|---|---|
| #1 | 9.4 m/s, 330° | 25.7 m/s |
| #2 | 9.5 m/s, 330° | 26.3 m/s |
| #3 | 9.3 m/s, 337° | 25.4 m/s |

Bu, daha önce −11 ile −13 saniyeye yol açan **tam o rüzgâr vektörüdür**. Tek
koşunun geçmesi kanıt değildir; zorlu koşulun yaşandığı loglardan doğrulanmıştır.

### Bekleme dağılımı (madde 8)

| Senaryo | Yerde | Havada |
|---|---|---|
| Sakin | 182 s | **0 s** |
| Değişken | 183 s | **0 s** |
| Sabit 8 m/s | 183 s | 62 s |

Beklemenin neredeyse tamamı yerde. Havada bekleme yalnızca sabit rüzgâr
senaryosunda ve yalnızca yasal kapı loiteri olarak ortaya çıkıyor.

## 4.3 Karşılaşılan Problemler

Bu bölüm, geliştirme sürecinde ölçülerek bulunan kök nedenleri içerir. Her biri
belirtiyi değil **sebebi** anlatır.

### Problem 1 — Ölçülen hıza bölmek (±15 s salınım)

**Belirti:** ETA sürekli salınıyor, hız komutu titriyor, plan yeniden kuruluyor.

**Kök neden:** ETA `kalan_mesafe / ölçülen_yer_hızı` ile hesaplanıyordu. Ölçülen
hız dönüşlerde anlık çöküyor.

**Sayısal örnek.** Kalan 2504 m, yer hızı 25.7 m/s:

$$\text{ETA}_{\text{düz}} = \frac{2504}{25.7} = 97.4\ \text{s}$$

Araç dönüşte bacak doğrultusundan 40° saparsa ilerleme hızı 19.7 m/s'ye düşer:

$$\text{ETA}_{\text{dönüşte}} = \frac{2504}{19.7} = 127.1\ \text{s}$$

**30 saniyelik hayalî gecikme** — araç aynı yerde, aynı hızda.

**Çözüm:** rüzgâr düzeltmeli model tabanlı hesap. **Salınım ±15 s → ±0.7 s.**

Bu tek düzeltme üç ayrı belirtiyi birden kapattı; hepsi aynı kök nedendendi.

### Problem 2 — Rüzgâr kestiriminin üç katmanlı hatası

**Katman A — mimari yanlış anlama.** Bizim hesabımızın cebirsel olarak EKF'in
kendi rüzgâr durumunu geri okuduğu geç fark edildi. Sonuç: filtreyi iyileştirmek
işe yaramıyor, **EKF'i ayarlamak** gerekiyor.

**Katman B — `ARSPD_USE = 0`.** Hava hızı sensörü EKF füzyonuna kapalıyken EKF
rüzgârı yalnızca GPS ve manevralardan çıkarmaya çalışıyordu.

> Ölçülen: SITL rüzgârı 5 m/s 45°'den iken kestirim **4.2 m/s 215°'den** — yön
> 170° ters.

**Katman C — `EK3_WIND_P_NSE = 0.1`.** EKF rüzgâr durumunu çok yavaş
güncelliyordu.

> Ölçülen: gerçek rüzgâr değişmişken kestirim 5.2 m/s 52°'de takılı kaldı; son
> 10 saniyede yer hızını 11'den 29 m/s'ye çıkaran arkadan rüzgârı görünmez yaptı.

**Çözüm:** `ARSPD_USE 1`, `EK3_WIND_P_NSE 1.0`.

### Problem 3 — Gövde→ENU dönüşünde pitch ihmali

**Belirti:** tırmanışta rüzgâr zayıf ölçülüyor.

**Kök neden:** hava hızı vektörü yalnızca yaw ile döndürülüyordu. AP_DDS'in
yayınladığı vektör ise EKF tarafından **tam yönelimle** (roll+pitch+yaw) gövde
çerçevesine döndürülmüş.

> Ölçülen: tırmanışta 8 m/s'lik rüzgâr **6.3 m/s** olarak kestiriliyordu (%21 hata).

**Çözüm:** tam kuaterniyon dönüşü.

### Problem 4 — Hız değişim limiti (%98.5 doygunluk)

**Belirti:** araç düzeltmeyi yetiştiremiyor; son bacakta erkenlik kapanmıyor.

**Kök neden:** `airspeed_rate_limit_mps2 = 0.5`. Kontrolcü uçuşun neredeyse
tamamında doygundu.

| Değer | `rate_limited=True` |
|---|---|
| 0.5 m/s² | **865 / 878 = %98.5** |
| 1.5 m/s² | 350 / 568 = %62 |

28'den 13'e inmek 0.5 m/s²'de **30 saniye ve ~615 metre** sürüyordu. Erkenlik
700 m kala fark edildiğinde düzeltme fiziksel olarak yetişemiyordu.

**Çözüm:** 0.5 → 1.5 m/s².

### Problem 5 — Bayat peer verisi (12 saniye)

**Belirti:** varmış bir aracın donmuş rüzgâr kestirimi hâlâ kullanılıyordu.

**Kök neden:** `wind_valid` bayrağı araç RTL'e girdikten sonra da doğru
kalıyordu.

**Maliyet:** 12 saniye zamanlama hatası.

**Çözüm:** yayınlanan alan tazelik bilgisi taşımalı — bayrak yalnızca araç
havadayken ve kestirim oturmuşken doğru.

### Problem 6 — Takipçinin kendi rüzgârıyla düzeltme yapması

**Belirti:** HA-2 nominal süresini 538 → 728 s yaptı, HA-2 − HA-1 arası **105
saniyeye** çıktı.

**Kök neden:** havadaki araç kestirimi oturur oturmaz **tek seferlik** düzeltme
yapıyor; tırmanış sırasında ölçülen rüzgâr henüz sapmalı ve bu değer
kalıcılaşıyor.

**Çözüm:** düzeltmeyi yalnızca öncü kendi ölçümünden yapar; takipçiler öncünün
taahhüdündeki kaymayı izler. Yerdeki araç ölçümü tekrar tekrar düzeltebildiği
için yakınsıyor.

### Problem 7 — Kendi testimizin gerçek dışı olması

**Bu, en pahalı problemdi** çünkü haftalarca **belirtiyi kovaladık.**

Doğrulama profilimiz rüzgârı **20 saniyede 110 derece** döndürüyordu:

$$\frac{110°}{20\ \text{s}} = 5.5\ °/\text{s}$$

Gerçek atmosferle karşılaştırma:

| Durum | Dönüş hızı |
|---|---|
| Sakin hava | < 0.1 °/s |
| Cephe geçişi | 0.2–0.5 °/s |
| Fırtına çıkış cephesi | 1.5–3 °/s |
| **Bizim profil** | **5.5 °/s, 12 dakikada dört kez** |

Yani "değişken rüzgâr" değil, **arka arkaya dört fırtına çıkış cephesi** simüle
ediyorduk. Gerçek harekâtta o koşulda uçuş iptal edilir.

**Bu profil yüzünden üç ayrı "çözüm" boşa gitti:**

| Deneme | Sonuç |
|---|---|
| Robust zarfı ölçülen rüzgâr etrafında daraltmak | Pencere %51→%87 açıldı, **zamanlama düzelmedi**, havada bekleme 44→115 s |
| Hız değişim limitini artırmak | Gerçek kusurdu (%98.5 doygunluk) ama **tek başına yetmedi** |
| S-manevrası eklemek | Üç koşuda da sırayı **bozdu** |

**Çözüm:** profili cephe geçişi seviyesine indirmek — 30° adım, `SIM_WIND_TC`
20→60 s, tepe **0.50 °/s**.

**Kritik nokta: test zayıflamadı.** Rüzgâr hızları (4–10 m/s, Beaufort 3–5)
değişmedi ve yeni profil son bacakta kuyruk rüzgârını **daha uzun süre**
üretiyor:

| Profil | Kuyruk rüzgârlı adım |
|---|---|
| Eski | 5 adımın 1'i |
| **Yeni** | 5 adımın **3'ü** |

Kaldırılan tek şey gerçek dışı dönüş hızıydı.

**Sonuç:** kod değişmeden, yalnızca profil gerçekçi hale getirilerek üç senaryo
da geçti.

### Ders

> Bir doğrulama senaryosu başarısızlık üretiyorsa, önce **senaryonun kendisinin
> fiziksel olarak savunulabilir olduğunu** doğrula.

## 4.4 S-Manevrası Neden Yok

Madde 5'in dört yönteminden biri. Uygulandı, ölçüldü, kaldırıldı. Karar
gerekçesi:

### Belge zorunlu tutmuyor

| Madde | İfade | Kip |
|---|---|---|
| 5 | *"şu yöntemleri **kullanabilir** veya daha farklı entegre bir teknik geliştirebilirsiniz"* | izin |
| 6 | *"S-manevrası gibi... teknikleri **kullanılabilirsiniz**"* | izin |
| 4 | *"S-manevrası gibi algoritmalar **için** ... 500m sapma"* | koşullu kısıt |

Üçü de izin kipinde. Kullandığımız üç yöntem (kalkış gecikmesi, rota
noktalarında loiter, dinamik seyir hızı) 20 s şartını üç senaryoda da sağlıyor.

### Yürütme çözüldü ve uçuşta doğrulandı

İlk iki deneme GUIDED moduyla yapıldı ve ikisinde de araç takıldı. Sebep
ArduPlane kaynağından bulundu:

```cpp
// ArduPlane/mode_guided.cpp:106
void ModeGuided::navigate() {
    plane.update_loiter(active_radius_m);   // GUIDED bir LOITER kontrolcusu
}
```

```cpp
// ArduPlane/commands.cpp:81,97 — set_guided_WP
prev_WP_loc = current_loc;        // bacak baslangicini simdiki konuma tasir
auto_state.crosstrack = false;    // "disable crosstrack, head directly to the point"
```

**GUIDED yol takibi yapmıyor.** Alternatif geliştirildi: kalkışta son bacağa boş
görev yuvaları konuluyor, gerektiğinde `MISSION_WRITE_PARTIAL_LIST` ile üzerine
yazılıyor. Görev silinmiyor, mod değişmiyor, AUTO'nun L1'i yolu gerçekten takip
ediyor.

**Uçuşta doğrulandı:** rota sapması **151 m** ölçüldü (planlanan 138 m, taban
~30 m). Araç yazılan noktaları takip etti, takılmadı.

### Ama geometri uçurulamadı

Manevra, araç erken olduğu için **asgari hızda** tetikleniyor. O hızda dar
dönüşlerde araç yol kaybediyor:

| Kalan | Hata | Komut | **Yer hızı** |
|---|---|---|---|
| 232 m | +7.1 s | 13.0 | **3.9** |
| 270 m | +19.0 s | 13.0 | **3.7** |
| 326 m | +37.0 s | 13.0 | **3.4** |

Kalan mesafe 232 → 365 m **arttı**: araç ~35 saniye hedeften uzaklaştı.

### Ve entegrasyon planlama katmanını kararsızlaştırdı

Sürekli yuva güncellemesi denendiğinde manevra yolu muhasebesi bozuldu:

```
rota kalan 4971 m | capa +4474.1 s | plan T+-4202 s
```

Araç hedefe **14 m** mesafedeyken model 4971 m kaldığını sanıyordu. Mandallı
çıpa 4474 saniyeye tırmandı, araç hedefin üzerinde daire çizdi ve görev bitmedi.

### Karar

Üç ardışık koşuda sıra bozuldu (+30, +77, +86 s). S-manevrası kaldırıldı:
**−1453 satır**. Yürütme mekanizmasının çalıştığı kanıtlandı ama geometri ve
entegrasyon çözülmedi.

Açılmadan önce gerekenler kodda kayıtlı: manevra yolu muhasebesi düzeltilmeli,
hız **taban** olmalı (kilit değil), çıpa mandalı şişen ETA'ya karşı korunmalı.

## 4.5 Doğrulama Tuzakları

Bu projede tekrar tekrar karşılaşılan ve ölçüm güvenilirliğini bozan durumlar:

| Tuzak | Belirti | Önlem |
|---|---|---|
| **Port yarışı** | "DDS oturumu kurulamadı" | Yeni koşudan önce `stop_all.sh` + bekleme |
| **`pkill -f` kendini vurur** | Kabuk aniden kapanır | `pgrep -f 'patt[e]rn' \| xargs -r kill` |
| **`grep -c` çift çıktı** | `"0\n0"` | `\|\| true` + boş kontrol |
| **Çalışma alanı kaynak alınmamış** | "Package not found" | Girişte `source install/setup.bash` |
| **Varış sonrası RTL örnekleri** | Yanlış "son yaklaşma" verisi | `ARRIVED` satırından önceye filtrele |
| **Faz hizalaması** | Tek koşu yanıltıcı | Zorlu vektörün yaşandığını loglardan doğrula |

Son madde önemlidir: değişken rüzgâr koşularında sonuç, profil enjeksiyonu ile
aracın son bacağa varışı arasındaki faz ilişkisine bağlıdır. **Tek geçen koşu
kanıt değildir**; zorlu koşulun gerçekten yaşandığı doğrulanmalıdır.

## 4.6 Geliştirme Yöntemi

Süreç boyunca izlenen ilkeler:

1. **Küçük, test edilebilir adım.** Her değişiklik ayrı ölçüldü.
2. **Kök neden, belirti değil.** "Şunu deneyelim" yerine "neden böyle oluyor".
3. **Geri alınan denemeler koda yazıldı.** Aynı yanlış iki kez denenmesin diye
   gerekçesi ve ölçümüyle birlikte yorum olarak bırakıldı.
4. **Kontrol koşusu olmadan sonuç yorumlanmadı.** S-manevrası değerlendirmesinde
   aynı profil hem açık hem kapalı koşuldu.
5. **Her sayı doğrulandı.** Bu raporda geçen ölçümler koddan ya da uçuş
   loglarından okundu.

## 4.7 Bilinen Eksikler

| Eksik | Durum |
|---|---|
| `FAILSAFE` işleyicisi | `safety_manager.py` **boş** (0 satır). Telemetri kesintisi, GPS kaybı, arm reddi için kurtarma yok. |
| S-manevrası | Kaldırıldı; açılma önkoşulları kodda kayıtlı. |
| Uç durum rüzgârı | 5.5 °/s dönüşte sapma −2.55 s. Algoritmanın ölçülmüş sınırı. |
| Split-brain | Ağ bölünürse iki araç ayrı takvim kurar; koruma yok. |
| Tek makine | `monotonic_ns` süreç yerel; gerçek dağıtık donanımda ortak zaman kaynağı gerekir. |

## 4.8 İlgili Sayfalar

- [01 - Sistem Mimarisi](01-sistem-mimarisi.md)
- [02 - Haberleşme Akışı](02-haberlesme-akisi.md)
- [03 - Algoritmalar](03-algoritmalar.md)
- [16 - Rüzgâr Profili ve Gerçekçilik](kavramlar/16-ruzgar-profili-ve-gercekcilik.md) — Problem 7'nin ayrıntısı
- [11 - Varış Zamanı Kontrolcüsü](kavramlar/11-varis-zamani-kontrolcusu.md) — Problem 4'ün ayrıntısı
- [04 - Plan Revizyonu](kavramlar/04-plan-revizyonu.md) — Problem 6'nın ayrıntısı
