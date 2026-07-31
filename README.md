<<<<<<< HEAD
# OASY — Merkeziyetsiz Çoklu İHA Zamanlanmış Varış Sistemi

> **Durum: taslak.** Bu depo şu anda yalnızca proje iskeletini içerir. Dosyaların
> içerikleri geliştirme aşamalarında doldurulacaktır.

Üç sabit kanatlı İHA'nın (ArduPlane 4.6.3 SITL) farklı kalkış noktalarından
otonom kalkarak ortak bir hedefe **HA-1 → HA-2 → HA-3** sırasıyla ve aralarında
**20 saniye** fark olacak şekilde varmasını sağlayan merkeziyetsiz ROS 2 sistemi.

Merkezi bir yer kontrol istasyonu veya master node kullanılmaz; her araç kendi
kararını diğer araçların yayınlarını dinleyerek bağımsız olarak verir.

## Kullanılan sürümler

| Bileşen | Sürüm |
|---|---|
| Ubuntu | 22.04.5 LTS |
| ROS 2 | Humble |
| Python | 3.10.12 |
| ArduPlane | 4.6.3 (tag `Plane-4.6.3`, commit `3fc7011a7d`) |
| Micro XRCE-DDS Agent | v2.4.2 |
| Micro XRCE-DDS Gen | v4.5.1 |

Otopilot ↔ ROS 2 köprüsü olarak MAVROS değil, ArduPilot'un yerleşik **AP_DDS**
(XRCE-DDS) altyapısı kullanılır.

## Dizin yapısı

| Yol | İçerik |
|---|---|
| `ros2_ws/src/oasy_interfaces/` | Özel ROS 2 mesajları (`VehicleStatus`, `ControlDecision`, `MissionEvent`) |
| `ros2_ws/src/oasy_uav_agent/` | Araç başına çalışan bağımsız karar node'u ve alt modülleri |
| `ros2_ws/src/oasy_bringup/` | Launch dosyaları, YAML konfigürasyonları, ArduPlane parametre dosyaları |
| `missions/` | Araç başına rota dosyaları (QGC WPL) |
| `scripts/` | SITL başlatma, tek tık başlatıcı, görev sonu analiz aracı |
| `tests/` | Birim, entegrasyon ve senaryo testleri |
| `docs/` | Teknik rapor, akış şemaları, algoritma açıklamaları, test sonuçları |
| `logs/` | Koşu bazlı loglar ve metrikler (versiyonlanmaz) |
| `wheelhouse/` | Teslim için `.whl` paketleri (versiyonlanmaz) |

## Kurulum, çalıştırma ve testler

Bu bölümler geliştirme ilerledikçe doldurulacaktır. Buraya yalnızca gerçek
sistem üzerinde çalıştırılıp doğrulanmış komutlar yazılacaktır.
=======
# multi-uav-arrival-coordination
ROS 2 Humble, ArduPlane SITL ve AP_DDS ile geliştirilen dağıtık çoklu sabit kanatlı İHA varış koordinasyonu.
>>>>>>> b6574ae68860063b3a19f5715c6d05d60cbda0bc
