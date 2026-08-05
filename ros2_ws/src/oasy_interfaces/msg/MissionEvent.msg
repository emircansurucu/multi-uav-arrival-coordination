# kritik görev olaylarının güvenilir kaydı

uint8 SEVERITY_INFO=0  # bilgi olayı
uint8 SEVERITY_WARN=1  # uyarı olayı
uint8 SEVERITY_ERROR=2  # hata olayı

uint8 EVENT_DDS_CONNECTED=0  # dds bağlantı olayı
uint8 EVENT_MAVLINK_CONNECTED=1  # mavlink bağlantı olayı
uint8 EVENT_MISSION_UPLOADED=2  # görev yükleme olayı
uint8 EVENT_TAKEOFF_SCHEDULED=3  # kalkış planlama olayı
uint8 EVENT_ARMED=4  # arm olayı
uint8 EVENT_TAKEOFF_STARTED=5  # kalkış başlama olayı
uint8 EVENT_ARRIVAL_COMMITTED=6  # varış taahhüdü olayı
uint8 EVENT_TERMINAL_ENTER=7  # terminal giriş olayı
uint8 EVENT_TARGET_REACHED=8  # hedefe varış olayı
uint8 EVENT_RTL_COMMANDED=9  # rtl komutu olayı
uint8 EVENT_RTL_CONFIRMED=10  # rtl doğrulama olayı
uint8 EVENT_PEER_LOST=11  # araç kaybı olayı
uint8 EVENT_FAILSAFE=12  # güvenli durum olayı

std_msgs/Header header  # olay zaman damgası

uint8 vehicle_id  # araç kimliği
uint64 monotonic_ns  # olayın tekdüze saat değeri

uint8 event_type  # olay türü
uint8 severity  # önem seviyesi
string detail  # olay açıklaması
