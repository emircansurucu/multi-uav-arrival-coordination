# Geliştirme ve Testler

Geliştirme önce haberleşme ayrımı, sonra görev yürütme, koordinasyon, rüzgâr hesabı ve terminal kontrolü sırasıyla yapıldı. Uçuş dışındaki hesaplar birim testlerle, bütün sistem ise SITL koşularıyla doğrulandı.

## Kabul ölçütleri

- varış sırası HA-1, HA-2 ve HA-3 olmalı
- ardışık varış farkı 20 s olmalı
- hedef geçişi 5 m içinde olmalı
- rota sapması 500 m altında kalmalı
- havada gereksiz bekleme en aza indirilmeli

Doküman zaman farkı için sayısal tolerans vermediği için teslim doğrulamasında ±1 s kullanıldı.

## Senaryolar

- sakin hava
- sabit 8 m/s rüzgâr
- değişken yön ve şiddette rüzgâr
- geliştirme sınırını görmek için ayrı extreme profil

Ana üç senaryoda ikişer başarılı koşu alındı. En büyük ardışık zaman hatası 0,17 s, en uzak hedef geçişi 4,99 m ve en büyük rota sapması 127 m oldu.

Son video koşusu değişken rüzgâr altında yapıldı:

| Ölçüm | Sonuç |
| HA-1 ile HA-2 | 20,07 s |
| HA-2 ile HA-3 | 19,94 s |
| havada bekleme | 0 s |
| en büyük rota sapması | 83 m |

## Birim testler

Uçuş gerektirmeyen geodesy, ETA, rüzgâr, planlama, peer yönetimi, kontrolcü, mission builder ve mission manager davranışları test edilir.

python3 -m pytest tests/unit -q

## Karşılaşılan temel problemler

### ETA salınımı

Kalan mesafeyi anlık ilerleme hızına bölmek dönüşlerde yaklaşık 30 s sahte gecikme üretti. Kontrol ETA'sı kalan rotayı bacak bacak hesaplayan modele taşındı.

### Rüzgâr ekseni

Hava hızını yalnız yaw ile döndürmek tırmanışta yanlış rüzgâr üretti. Tam quaternion dönüşümü kullanıldı.

### Hız rampası

İlk 0,5 m/s² sınır kontrolcüyü neredeyse sürekli doyuma soktu. Ölçümden sonra değer 1,5 m/s² yapıldı.

### Bayat peer bilgisi

Görevini bitiren veya iletişimi kesilen aracın eski rüzgâr verisinin kullanılmasını önlemek için tazelik ve görev durumu filtresi eklendi.

### Son yaklaşmada kontrol yetkisi

Yalnız en erken varışı izlemek yeterli olmadı. En geç ulaşılabilir varış, son yasal kapı ve terminal rezerv mekanizmaları eklendi.

### S-manevrası

İlk geometri farklı referanslarla planlanıp ölçüldüğü için rota sapması büyüdü. Referans düzeltildi ancak GUIDED davranışı ve ETA entegrasyonu sistemi gereksiz karmaşıklaştırdı. Vaka S-manevrasını zorunlu tutmadığı için nihai çözümde kullanılmadı.

