# DDS Mimarisi ve Domain Ayrımı

AP_DDS konu adları araç kimliğiyle ayrılmadığı için üç SITL aynı domainde çalıştırılmaz.

| Araç | DDS domain | XRCE portu |
| HA-1 |     1      |   2019     |
| HA-2 |     2      |   2020     |
| HA-3 |     3      |   2021     |

Koordinasyon mesajları ortak domain 10 üzerinde yayınlanır. Her agent süreci iki ayrı `rclpy.Context` ve iki executor kullanır:

- araç contexti yalnız kendi AP_DDS telemetrisini alır
- koordinasyon contexti bütün `VehicleStatus` mesajlarını alır ve yayınlar

Başlangıçta telemetri konumu beklenen pist konumuyla karşılaştırılır. Yanlış domaine bağlanan araç görev başlatılmadan reddedilir.

Kod konumu:
- `agent_node.py`
- `coordination/status_publisher.py`
- `oasy_bringup/config/ha*.yaml`
