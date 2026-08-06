# araçların birbirine yayınladığı durum mesajı

uint8 STATE_INIT=0  # başlangıç durumu
uint8 STATE_CONNECTING=1  # bağlantı durumu
uint8 STATE_MISSION_UPLOAD=2  # görev yükleme durumu
uint8 STATE_WAIT_PEERS=3  # diğer araçları bekleme durumu
uint8 STATE_WAIT_TAKEOFF_SLOT=4  # kalkış anını bekleme durumu
uint8 STATE_ARMING=5  # arm durumu
uint8 STATE_TAKEOFF=6  # kalkış durumu
uint8 STATE_CLIMB=7  # tırmanış durumu
uint8 STATE_CRUISE=8  # seyir durumu
uint8 STATE_TERMINAL=9  # terminal durumu
uint8 STATE_ARRIVED=10  # varış durumu
uint8 STATE_RTL=11  # rtl durumu
uint8 STATE_DONE=12  # tamamlanma durumu
uint8 STATE_FAILSAFE=13  # güvenli durum

std_msgs/Header header  # kayıt ve video zaman damgası

uint8 vehicle_id  # araç kimliği
uint32 seq  # mesaj sıra numarası
uint8 mission_state  # görev durumu

uint64 monotonic_ns  # zamanlama için tekdüze saat değeri

float64 latitude  # enlem
float64 longitude  # boylam
float32 altitude_msl  # msl irtifası
float32 groundspeed  # yer hızı

float32 remaining_distance  # kalan rota mesafesi
float32 eta_seconds  # tahmini kalan süre

uint64 planned_arrival_monotonic_ns  # taahhüt edilen varış anı
bool arrival_committed  # varış planı kilit durumu

uint64 earliest_feasible_arrival_monotonic_ns  # ulaşılabilen en erken varış anı

bool wind_valid  # rüzgar ölçüm geçerliliği
float32 wind_speed  # rüzgar hızı
float32 wind_dir_deg  # rüzgarın geldiği yön

bool target_reached  # hedefe varış durumu
uint64 actual_arrival_monotonic_ns  # gerçek varış anı
