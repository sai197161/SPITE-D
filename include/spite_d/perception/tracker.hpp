#pragma once

// Multi-object tracking interface for the perception module.
//
// The first implementation ports map_manager's UV-detector + Kalman
// tracker (OpenCV/Eigen internals, originally ROS1) behind this
// interface. The node shim feeds it RGB-D frames; everything below the
// shim is ROS-free and testable offline from recorded frames.

#include "spite_d/common/types.hpp"

#include <vector>

namespace spite_d {

/// Minimal depth-frame handoff: row-major uint16 depth in millimeters
/// plus the intrinsics needed to deproject. Matches what a
/// sensor_msgs/Image (16UC1) + CameraInfo pair carries.
struct DepthFrame {
  int width{0};
  int height{0};
  double fx{0}, fy{0}, cx{0}, cy{0};
  double stamp{0.0};
  Pose3 cameraPose;  ///< Camera-to-world transform at this stamp.
  const uint16_t* depth{nullptr};  ///< Borrowed; valid for the call only.
};

class Tracker {
 public:
  virtual ~Tracker() = default;

  /// Ingest one frame and return the current set of tracked obstacles
  /// (world frame, with velocities and covariances).
  virtual std::vector<TrackedObstacle> Update(const DepthFrame& frame) = 0;
};

}  // namespace spite_d
