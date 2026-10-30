# Standard Path几何的身份绑定元信息；不携带第二份路径。
uint8 FOLLOW=0
uint8 BRAKE=1
uint8 HOLD=2
uint8 FINAL_ALIGN=3
uint8 INVALIDATED=4
std_msgs/Header header
unique_identifier_msgs/UUID session_id
uint64 reference_revision
string path_digest
uint8 intent
float64 position_tolerance_m
float64 yaw_tolerance_rad
float64 stopped_linear_mps
float64 stopped_angular_rps
