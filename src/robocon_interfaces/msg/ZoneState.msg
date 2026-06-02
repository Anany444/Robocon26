# ZoneState.msg
# Defines the current zone status of the robot.

string global_zone     # e.g., "meihua_forest", "martial_club", "arena"
string local_zone      # e.g., "block_5", "ramp", "r2_start_zone", "none"
float32 expected_height # e.g., 0.0, 0.2, 0.4, 0.6
bool on_slope          # True if the robot is currently on the ramp
