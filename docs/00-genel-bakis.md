# OASY Genel Bakış

Bu proje üç ArduPlane 4.6.3 SITL aracının ortak hedefe HA-1, HA-2 ve HA-3 sırasıyla 20 saniye arayla varmasını sağlar. Her araç kendi kararını verir. Merkezi bir görev yöneticisi veya yer kontrol istasyonu kullanılmaz.

## Temel yaklaşım

- araçlar görevlerini AUTO modunda yürütür
- rota ve görev MAVLink ile yüklenir
- konum, hız, yönelim ve hava hızı AP_DDS üzerinden alınır
- araçlar koordinasyon bilgisini ortak DDS domaininde paylaşır
- uzun bekleme yerde kalkış gecikmesiyle yapılır
- uçuş sırasında küçük zaman hataları hava hızıyla düzeltilir
- hedefe 2 km kala terminal kontrolü uygulanır
- varış hedef merkezindeki 5 m çembere giriş anıdır

## Proje yapısı

- `ros2_ws/src/oasy_uav_agent`: görev ve koordinasyon kodu
- `ros2_ws/src/oasy_interfaces`: araçlar arası ROS 2 mesajları
- `ros2_ws/src/oasy_bringup`: araç ayarları ve launch dosyası
- `scripts`: başlatma, durdurma, rüzgâr, analiz ve video araçları
- `tests`: uçuş gerektirmeyen birim testleri
- `docs`: kısa teknik notlar
- `rapor`: teslim edilecek teknik rapor

## Çalıştırma

```bash
./run.sh
./run.sh steady
./run.sh variable
```
Son koşuyu incelemek için:

```bash
python3 scripts/analyze_run.py --log logs/run_YYYYMMDD_HHMMSS/agents.log
```
