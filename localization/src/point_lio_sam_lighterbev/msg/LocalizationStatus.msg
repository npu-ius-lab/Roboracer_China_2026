std_msgs/Header header

uint8 WAITING_FOR_MAP=0
uint8 RELOCALIZING=1
uint8 TRACKING=2
uint8 DEGRADED=3

uint8 state
bool map_loaded
bool localized
bool frontend_healthy
bool global_probe_active
float64 registration_score
float64 correspondence_ratio
float64 mean_residual
float64 map_odom_translation_update
float64 map_odom_rotation_update_deg
int32 consecutive_failures
int32 relocalization_confirmations
int32 matched_keyframe
string reason
