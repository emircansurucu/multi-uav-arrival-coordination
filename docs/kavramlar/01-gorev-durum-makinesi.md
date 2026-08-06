# Görev Durum Makinesi

Görev akışı tek yönlü 14 ana durumdan oluşur:

INIT -> CONNECTING -> MISSION_UPLOAD -> WAIT_PEERS
-> WAIT_TAKEOFF_SLOT -> ARMING -> TAKEOFF -> CLIMB
-> CRUISE -> TERMINAL -> ARRIVED -> RTL -> DONE

Görev threadinde yakalanmayan hata oluşursa `FAILSAFE` durumuna geçilir.

Her kontrol çevriminde yalnız mevcut durumun işlemi çalışır. Tamamlanma koşulu sağlanınca sonraki duruma geçilir. Böylece mission upload, arm ve RTL gibi tek seferlik işlemler yanlışlıkla tekrarlanmaz.

Hazırlık durumları bağlantıları ve görev yüklemeyi, uçuş durumları zaman kontrolünü, tamamlama durumları varış ve RTL doğrulamasını yürütür.

Kod konumu:

- `oasy_uav_agent/mission_manager.py`
- `oasy_interfaces/msg/VehicleStatus.msg`
