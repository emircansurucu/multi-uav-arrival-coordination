# S-manevrası — durum devri

Bu belge, S-manevrasının neden hâlâ kapalı olduğunu ve çözülmesi gereken iki
somut problemi anlatır. Sistemin geri kalanı çalışır durumdadır.

## 1. Bağlam

**Proje:** ArduPlane 4.6.3 SITL üzerinde üç sabit kanatlı İHA, ROS 2 Humble,
AP_DDS (MAVROS yok). Üç araç farklı kalkış noktalarından ortak bir hedefe
**HA-1 → HA-2 → HA-3** sırasıyla ve aralarında **20 saniye** farkla varmalı.
Merkezi karar düğümü yok; her araç kendi kararını peer yayınlarını dinleyerek
verir.

**S-manevrasının rolü:** Zaman kaybetmenin dört yolu var ve her biri farklı
bir yerde çalışır:

| Kaldıraç | Nerede | Sınır |
|---|---|---|
| Kalkış geciktirme | Sadece yerde | Sınırsız |
| Hız düşürme | Her yerde | `(1 − v_min/v_seyir) × kalan süre` |
| Loiter | Hedefe **>2 km** | Dokümanda 2 km içinde YASAK |
| **S-manevrası** | Her yerde, **2 km içi dahil** | Rotadan ≤500 m sapma |

S, hedefin 2 km'si içinde zaman kaybettirebilen **tek** yöntemdir. İhtiyaç
ölçümle doğrulanmıştır: 8 m/s rüzgârda HA-3 terminal fazda minimum hava
hızına (13 m/s) dayanıp hâlâ ~18 s erken varıyor.

**Not:** Doküman S-manevrasını zorunlu kılmıyor ("kullanabilirsiniz"). Ama
değişken rüzgâr şartı (madde 7) geç gelen rüzgâr değişimlerini kapsıyor ve o
durumda geriye tek yasal kaldıraç S kalıyor.

## 2. Ne yapıldı

### Planlama tarafı — çalışıyor, 19 test geçiyor

`ros2_ws/src/oasy_uav_agent/oasy_uav_agent/control/maneuver_planner.py`

Zikzak geometrisi. Bacak uzunluğu `L`, eklenmek istenen mesafe `e`, tek diş
için yanal ofset `d`:

```
2·√((L/2)² + d²) = L + e   ⇒   d = √(e·(2L + e)) / 2
```

Bacağı N döngüye bölmek hem `L`'yi hem `e`'yi böldüğü için gereken yanal
ofseti hızla küçültür: 1304 m'lik bacakta +280 m için tek döngü 450 m, üç
döngü 150 m ister. Planlayıcı sapmayı en aza indiren döngü sayısını seçer;
üst sınırı dönüş yarıçapı koyar (çok sıkı zikzağı otopilot keser).

`follow_path()` — GUIDED için kayan takip noktası (carrot) üretir.

### Yürütme tarafı — üç deneme

**Deneme 1: doğrudan waypoint komutu, 80 m yaklaşınca ilerlet.**
Başarısız. `AP_ExternalControl_Plane::set_global_position` yorumu:
*"Sets the target global position for a loiter point"* — ArduPlane GUIDED'da
komut edilen nokta bir **loiter merkezi**. SITL `plane.parm` içinde
`WP_LOITER_RAD 80`; ilerleme eşiğim de 80 m olduğu için mesafe hiçbir zaman
eşiğin altına inmedi. Ayrıca araç 2 km içinde çember attı → **madde 6 ihlali**.

**Deneme 2: carrot (takip noktası), 250 m ileride.**
Başarısız. `S-MANEVRASI BITTI` hiç görünmedi, araçlar takıldı.

**Deneme 3: `header.frame_id = "map"` düzeltmesi.**
`AP_DDS_ExternalControl.cpp:25`:

```cpp
if (strcmp(cmd_pos.header.frame_id, MAP_FRAME) == 0) {   // MAP_FRAME = "map"
```

`frame_id` boş bırakıldığında fonksiyon anında `false` döner ve **hedef hiç
ayarlanmaz**. Araç GUIDED'a geçer ama komut almaz, bulunduğu yerde çember
atar. Deneme 1 ve 2'nin başarısızlığını bu açıklıyor.

Düzeltmeden sonra **manevralar tamamlanıyor** (`S-MANEVRASI BITTI` üç kez).

## 3. Kalan iki problem

### Problem A — S rotayı kesiyor, 500 m sınırı aşılıyor (ÇÖZÜLDÜ: teşhis)

**Teşhis kesin.** İki bağımsız ölçüm:

```
S-MANEVRASI: yanal sapma  91 m ve 53 m  (planlanan)
HEDEFE VARILDI: max rota sapmasi 812 m  (olculen)
```

91 m plandan 812 m çıkmaz; takip taşması bunu açıklayamaz (9 kat).

Saf geometri hesabı nedeni veriyor: HA-1'in WP3→WP4 bacağı üzerinde, hedefe
2 km kalan bir noktadan **hedefe düz gidilirse**, manevra ofseti sıfır olsa
bile o bacaktan **3215 m** sapılır.

Kök neden: `plan_s_maneuver(position, target, ...)` çağrısı S'i **mevcut
konumdan hedefe düz çizgi** olarak planlıyor ve aradaki waypoint'leri
atlıyor. Manevra TERMINAL girişinde (hedefe 2 km) tetikleniyor; HA-1 o anda
henüz son bacakta değil, aktif bacağı WP3→WP4. Sapma ise aktif bacağa göre
ölçülüyor (`_track_route_deviation`). Yani sapma manevradan değil, **rotanın
kesilmesinden** geliyor.

**Çözüm yönü:** S, kalan rota poligonunu (konum → aktif WP → ... → hedef)
izlemeli; yanal ofsetler bu poligona göre uygulanmalı. O zaman planlanan ofset
ile ölçülen sapma aynı referansı paylaşır ve 500 m sınırı gerçekten bağlayıcı
olur. `plan_s_maneuver` şu an (start, end) düz bacak alıyor; poligon alacak
şekilde genelleştirilmeli.

### Problem B — manevra saçma büyüklükte tetikleniyor

`3016 m ek mesafe`, `333.3 s erken`. Bu bir manevra sorunu değil, **yukarı
akıştaki zamanlama sorunu**: bir araç 333 saniye erken kalmışsa plan/çapa
zaten bozulmuş demektir. S bunu maskeliyor, çözmüyor.

Bu koşuda plan hataları: HA-1 +162.85 s, HA-2 +198.08 s, HA-3 +12.48 s.
S kapalıyken aynı senaryoda hatalar +19.18 / +20.90 / +2.17 s idi. Yani S
açıldığında zamanlama belirgin biçimde bozuluyor — manevra ile zamanlama
mekanizmaları arasında hâlâ çözülmemiş bir etkileşim var.

## 4. Denenmiş ve işe yaramış düzeltmeler (geri alınmamalı)

- `header.frame_id = "map"` — yoksa komut sessizce yok sayılır
- Ulaşılabilirlik dondurma: manevra sırasında `_update_feasible_arrival`
  erken döner. Manevra ilerlemeyi düşürür ama aracın **yapabileceğini**
  değiştirmez; ölçülen değeri kullanmak çapayı geriye itip daha fazla manevra
  gerektiriyordu (pozitif geri besleme). Bu düzeltmeden sonra üretilen
  manevralar makul büyüklüğe indi (2219 m → 126–469 m).
- Son yaklaşma AUTO'ya devredilir (`MANEUVER_HANDOVER_M = 300 m`). Hedefin
  kendisi GUIDED'da komut edilmez; edilirse araç hedef etrafında çember atar
  ve 5 m kabul yarıçapına hiç giremez.

## 5. Ölçülmüş referans değerler

8 m/s kuzey rüzgârı (`SIM_WIND_SPD 8`, `SIM_WIND_DIR 0`, `SIM_WIND_T` karekök,
`SIM_WIND_T_ALT 60`):

| Yapılandırma | Sıra | T2−T1 sapma | T3−T2 sapma | Max sapma |
|---|---|---|---|---|
| S kapalı (mevcut) | DOĞRU | +1.72 s | −18.73 s | 150 m |
| S açık | YANLIŞ | +35.24 s | −185.61 s | **816 m** |

Rüzgârsız, S kapalı: sıra doğru, −0.72 s / +0.09 s.

Diğer sabitler: seyir 22.9 m/s, hız aralığı 13–28 m/s
(`AIRSPEED_MIN 10`, `AIRSPEED_MAX 30`), tırmanış ~200 s, `WP_LOITER_RAD 80`.

## 6. Açık sorular

Problem A'nın teşhisi tamam, uygulanacak değişiklik belli. Kalanlar:

1. S kalan rota poligonuna uygulanırken, poligonun **hangi kısmına** zikzak
   konmalı? Yalnızca aktif bacağa mı, yoksa hedefe kadar tüm kalan bacaklara
   mı? Aktif bacak kısaysa (son bacak 1304 m) yeterli mesafe çıkmayabilir.
2. ArduPlane'de S için GUIDED + `cmd_gps_pose` doğru araç mı, yoksa AUTO
   mission'a waypoint enjekte etmek mi daha sağlam? Mission yeniden yazmanın
   indeks kayması riski var ama waypoint'ler geçiştir, loiter değil.
3. Problem B tam çözüldü mü? Debounce sonrası manevra büyüklükleri makul
   (130 m, 73 m) ama zamanlama hâlâ bozuluyor (plan hatası +163 s). Bunun
   Problem A'nın yan etkisi mi (rota kesildiği için mesafe/ETA tutarsızlığı)
   yoksa ayrı bir sorun mu olduğu doğrulanmalı.

**Eksik enstrümantasyon:** carrot konumu hiç loglanmıyor. Manevra sırasında
komut edilen nokta ile aracın konumu kaydedilse yürütme hataları çok daha
hızlı ayırt edilirdi.

## 7. İlgili dosyalar

```
ros2_ws/src/oasy_uav_agent/oasy_uav_agent/
  control/maneuver_planner.py       geometri + follow_path (19 test)
  autopilot_adapter/dds_commands.py GUIDED konum komutu (frame_id burada)
  mission_manager.py                tetikleme, yurutme, sapma olcumu
  estimation/geodesy.py             cross_track_distance_m
ros2_ws/src/oasy_bringup/config/ha*.yaml   s_maneuver_enabled (su an false)
tests/unit/test_maneuver_planner.py
```

ArduPilot tarafı: `libraries/AP_DDS/AP_DDS_ExternalControl.cpp`,
`ArduPlane/AP_ExternalControl_Plane.cpp`.
