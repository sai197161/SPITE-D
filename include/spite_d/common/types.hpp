#pragma once

// Core types shared across spite_d modules.
//
// These are deliberately ROS-free and Eigen-free plain structs so every
// module core can be built and unit-tested on any machine with a C++17
// compiler. Conversion to/from ROS messages happens only in the node
// shims (src/nodes/), and conversion to open_spite types happens only
// in the spite module.

#include <array>
#include <cstdint>
#include <vector>

namespace spite_d {

using Mat3 = std::array<std::array<double, 3>, 3>;
using Vec3 = std::array<double, 3>;

/// Rigid-body pose: rotation + translation. Layout-compatible with
/// open_spite::Transformation3d (same member order and types).
struct Pose3 {
  Mat3 rotation{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  Vec3 translation{0, 0, 0};
};

/// One tracked obstacle at a single timestamp, as produced by the
/// perception module (detector + tracker).
struct TrackedObstacle {
  int32_t id{-1};          ///< Stable track id across frames.
  double stamp{0.0};       ///< Seconds; source clock is the sensor's.
  Pose3 pose;              ///< OBB center pose in the world frame.
  Vec3 halfExtents{0, 0, 0};  ///< OBB half-sizes along its local axes.
  Vec3 velocity{0, 0, 0};     ///< Estimated world-frame linear velocity.
  /// Position uncertainty (1-sigma, meters) per world axis. The basic
  /// tracker fills this from its Kalman covariance diagonal.
  Vec3 positionStd{0, 0, 0};
};

/// A predicted future trajectory for one tracked obstacle. Produced by
/// the trajectory module; consumed by the spite module (and later by
/// span construction in dynamic_map).
struct PredictedTrajectory {
  int32_t id{-1};
  Vec3 halfExtents{0, 0, 0};  ///< OBB half-sizes, assumed constant over the horizon.
  std::vector<double> stamps;  ///< Absolute times, strictly increasing.
  std::vector<Pose3> poses;    ///< OBB center pose at each stamp.
  /// Predicted position uncertainty (1-sigma, meters) at each stamp.
  /// Grows with lookahead for any real predictor; the fixed-horizon
  /// scheme is recovered by leaving these at zero.
  std::vector<Vec3> positionStd;
};

}  // namespace spite_d
