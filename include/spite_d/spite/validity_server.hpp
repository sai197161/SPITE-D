#pragma once

// ValidityServer: the bridge between perception-driven obstacle
// predictions and open-spite's DynamicRoadmapTool.
//
// Baseline (no spans): every perception update, each obstacle's
// predicted trajectory over the horizon is collapsed into SPITE
// obstacle geometry --
//   over-approximation:  OBB of the predicted OBB corners across all
//                        prediction samples, inflated by +k*sigma;
//   under-approximation: spheres inscribed in the obstacle placed along
//                        the predicted center path, deflated by -k*sigma
// -- then fed to DynamicRoadmapTool::UpdateObstacleOBB/Spheres +
// MarkDirty + ShallowUpdate. This recomputes obstacle geometry every
// frame; the span representation in dynamic_map will later amortize
// exactly this step, so keep this path intact as the ablation baseline.
//
// Gray elements can optionally be resolved with a fine collision check
// (fcl) or left lazy for the replanner to resolve on demand.

#include "spite_d/common/types.hpp"

#include <map>
#include <memory>
#include <utility>
#include <vector>

class DynamicRoadmapTool;  // open-spite

namespace spite_d {

class ValidityServer {
 public:
  enum class Validity { VALID, UNKNOWN, INVALID };

  struct Params {
    double sigmaGain{0.0};   ///< k in the +/- k*sigma inflation/deflation.
    bool resolveGray{true};  ///< Fine-check gray elements immediately vs lazily.
  };

  /// Takes ownership of a DynamicRoadmapTool that has already been
  /// built (Build() or ReadGeometriesFromFile()).
  ValidityServer(std::unique_ptr<DynamicRoadmapTool> drm, Params params);
  ~ValidityServer();

  /// Ingest this frame's predictions and update roadmap validity.
  /// Tracks appearing for the first time are registered; missing tracks
  /// are treated as unchanged (obstacle removal handled via Forget()).
  void Update(const std::vector<PredictedTrajectory>& predictions);

  /// Drop a track (target lost / left the workspace) and re-validate
  /// the roadmap elements it was invalidating.
  void Forget(int32_t obstacleId);

  Validity GetVertexValidity(size_t vid) const;
  Validity GetEdgeValidity(size_t src, size_t tgt) const;

  /// Edges currently classified INVALID or UNKNOWN, for the replanner.
  std::vector<std::pair<size_t, size_t>> BlockedEdges() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace spite_d
