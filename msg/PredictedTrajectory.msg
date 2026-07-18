# Predicted future trajectory of one tracked obstacle.
# stamps/poses/position_std are parallel arrays.
std_msgs/Header header
int32 id
geometry_msgs/Vector3 half_extents   # constant over the horizon
builtin_interfaces/Time[] stamps
geometry_msgs/Pose[] poses
geometry_msgs/Vector3[] position_std
