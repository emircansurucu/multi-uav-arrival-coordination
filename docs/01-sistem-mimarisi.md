# Sistem Mimarisi

Sistem üç ArduPlane SITL, üç Micro XRCE-DDS Agent ve üç araç agentından oluşur. Her araç agentı aynı yazılımı kendi yapılandırmasıyla çalıştırır.

## Araç tarafı

ArduPlane uçuş dinamiğini, rota takibini ve düşük seviye kontrolü yürütür. Python tarafı servo veya uçuş yüzeyi komutu üretmez. Yalnız görev yükleme, hız seçimi, zamanlama ve görev durumu yönetilir.

Her araç için iki iletişim yolu vardır:
- AP_DDS telemetri yolu
- MAVLink görev ve komut yolu

## DDS ayrımı

Araç telemetrileri birbirine karışmasın diye her SITL farklı domainde çalışır:

| Araç | DDS domain | XRCE UDP portu |
| HA-1 | 1 | 2019 |
| HA-2 | 2 | 2020 |
| HA-3 | 3 | 2021 |

Araçlar arası koordinasyon domain 10 üzerinden yapılır. Her agent aynı süreçte iki ayrı `rclpy.Context` kullanır. Bir context kendi aracını, diğeri ortak koordinasyon ağını dinler.

## Merkeziyetsiz çalışma

Her araç aşağıdaki bilgileri yayınlar:

- kimlik ve görev durumu
- planlanan varış zamanı
- en erken ulaşılabilir varış
- konum ve kalan rota
- geçerli rüzgâr kestirimi

Bütün agentlar aynı araç listesini ve aynı hesap kurallarını kullandığı için ortak zamanlama çıpasını bağımsız olarak bulur. HA-1 diğer araçlara komut veren bir master değildir.

## Yazılım katmanları

- `agent_node.py`: ROS context ve executor yönetimi
- `mission_manager.py`: görev durumları ve kontrol döngüsü
- `autopilot_adapter`: AP_DDS ve MAVLink bağlantıları
- `estimation`: rüzgâr, ETA, jeodezi ve varış hesabı
- `coordination`: peer verisi ve ortak plan
- `control`: hava hızıyla zaman düzeltmesi

## Görev akışı

INIT -> CONNECTING -> MISSION_UPLOAD -> WAIT_PEERS
-> WAIT_TAKEOFF_SLOT -> ARMING -> TAKEOFF -> CLIMB
-> CRUISE -> TERMINAL -> ARRIVED -> RTL -> DONE

Kontrol döngüsü 20 Hz çalışır. ROS executorları mesaj alıp yayınlar, görev kararları ayrı mission threadinde hesaplanır.
