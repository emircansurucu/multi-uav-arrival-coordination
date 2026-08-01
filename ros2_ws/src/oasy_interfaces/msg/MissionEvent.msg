# Kritik gorev olaylarinin guvenilir kaydi. Gec baslayan bir peer'in de gecmis
# olaylari gorebilmesi icin transient-local yayinlanir.
# QoS: RELIABLE, depth 20, TRANSIENT_LOCAL.
# detail alani ayni zamanda simulasyon videosundaki altyazi metnidir.

uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARN=1
uint8 SEVERITY_ERROR=2

uint8 EVENT_DDS_CONNECTED=0
uint8 EVENT_MAVLINK_CONNECTED=1
uint8 EVENT_MISSION_UPLOADED=2
uint8 EVENT_TAKEOFF_SCHEDULED=3
uint8 EVENT_ARMED=4
uint8 EVENT_TAKEOFF_STARTED=5
uint8 EVENT_ARRIVAL_COMMITTED=6
uint8 EVENT_TERMINAL_ENTER=7
uint8 EVENT_TARGET_REACHED=8
uint8 EVENT_RTL_COMMANDED=9
uint8 EVENT_RTL_CONFIRMED=10
uint8 EVENT_PEER_LOST=11
uint8 EVENT_FAILSAFE=12

std_msgs/Header header

uint8 vehicle_id
uint64 monotonic_ns

uint8 event_type
uint8 severity
string detail
