# One tracked obstacle at a single timestamp (world frame).
std_msgs/Header header
int32 id                       # stable track id
geometry_msgs/Pose pose        # OBB center pose
geometry_msgs/Vector3 half_extents
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 position_std   # 1-sigma position uncertainty per axis
