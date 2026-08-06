# Haberleşme Akışı

Sistem AP_DDS, MAVLink ve ortak ROS 2 koordinasyon ağı olmak üzere üç iletişim katmanı kullanır.

## ArduPilot tarafından alınan veriler
| `/ap/navsat` | enlem, boylam ve MSL irtifa |
| `/ap/twist/filtered` | ENU yer hızı |
| `/ap/pose/filtered` | konum ve quaternion yönelim |
| `/ap/airspeed_vector` | gövde eksenindeki hava hızı |

Telemetri örnekleri aynı araca ait olmalı ve zamanları birbirine yakın olmalıdır. Eski veya uyumsuz örnek kontrol hesabına alınmaz.

## ArduPilot tarafına gönderilenler

MAVLink aşağıdaki işlemler için kullanılır:

- görevi temizleme ve waypoint listesini yükleme
- AUTO ve GUIDED mod geçişleri
- arm işlemi
- `MAV_CMD_DO_CHANGE_SPEED` ile hava hızı ayarı
- görev sonunda RTL komutu

Görev yükleme sırasında mission request mesajları izlenir ve son `MISSION_ACK` beklenir. Mod değişiklikleri heartbeat üzerinden doğrulanır.

GUIDED konum komutu yalnız son yasal kapıda bekleme gerektiğinde `/ap/cmd_gps_pose` üzerinden kullanılır.

## Araçlar arası mesaj

`VehicleStatus.msg` ortak domain 10 üzerinde yayınlanır. Mesaj şu bilgi gruplarını taşır:

- araç kimliği, sıra numarası ve görev durumu
- monotonic zaman ve planlanan varış
- en erken ulaşılabilir varış
- konum, kalan rota ve tahmini varış süresi
- rüzgâr vektörü ve geçerlilik bilgisi
- hedefe varış ve RTL durumu

`MissionEvent.msg` kaynakta tanımlıdır ancak mevcut görev akışında kullanılmaz.

## QoS ve tazelik
Araç durumları hızlı değiştiği için best effort QoS kullanılır. Eski bir paketin yeniden gelmesi, yeni durum paketinden daha yararlı değildir.

Peer verisi yaşına göre sınıflandırılır:
- 0 ile 2 s arası güncel
- 2 ile 5 s arası eski
- 5 s üzeri kayıp

Sıra numarası geriye giden paketler reddedilir. Kayıp veya görevi tamamlanmış araç rüzgâr kaynağı olarak kullanılmaz.

## Tek kontrol çevrimi

1. güncel telemetri snapshotı alınır
2. peer durumları güncellenir
3. görev durumu ilerletilir
4. rota, rüzgâr ve ETA hesaplanır
5. gerekiyorsa hız veya mod komutu gönderilir
6. yeni `VehicleStatus` yayınlanır

MAVLink komutları ACK veya heartbeat ile doğrulanırken hızlı durum yayını kayıp toleranslıdır.
