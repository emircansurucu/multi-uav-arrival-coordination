# Araclarin birbirine yayinladigi durum sozlesmesi.
# Uc arac ayni konuya yazar, ayrim vehicle_id ile yapilir.
# QoS: BEST_EFFORT, depth 10, ~5 Hz.

uint8 STATE_INIT=0
uint8 STATE_CONNECTING=1
uint8 STATE_MISSION_UPLOAD=2
uint8 STATE_WAIT_PEERS=3
uint8 STATE_WAIT_TAKEOFF_SLOT=4
uint8 STATE_ARMING=5
uint8 STATE_TAKEOFF=6
uint8 STATE_CLIMB=7
uint8 STATE_CRUISE=8
uint8 STATE_TERMINAL=9
uint8 STATE_ARRIVED=10
uint8 STATE_RTL=11
uint8 STATE_DONE=12
uint8 STATE_FAILSAFE=13

# stamp wall-clock tasir, yalnizca kayit ve video icindir.
std_msgs/Header header

uint8 vehicle_id
# Peer mesaj kaybi bu sayacin atlamalarindan olculur.
uint32 seq
uint8 mission_state

# Butun zamanlama matematigi bu alan uzerinden yurur.
uint64 monotonic_ns

# float64 zorunlu: hedef koordinati sekiz anlamli basamak tasiyor.
float64 latitude
float64 longitude
float32 altitude_msl
float32 groundspeed

# Duz cizgi degil, kalan rota segmentlerinin toplami.
float32 remaining_distance
float32 eta_seconds

# Goreli sure yerine mutlak an: mesaj gecikmesine bagisik.
# Peer'lar yalnizca taahhut edilmis degeri referans alir.
uint64 planned_arrival_monotonic_ns
bool arrival_committed

bool target_reached
uint64 actual_arrival_monotonic_ns
