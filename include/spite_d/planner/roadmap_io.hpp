#pragma once

// Roadmap graph persistence.
//
// The offline build_roadmap tool writes two files:
//   - the graph (this module): vertex states + edges with costs, which the
//     replanner needs and open-spite's WriteGeometries does not store;
//   - the geometries (open-spite's format): OBB/spline approximations,
//     reloaded via DynamicRoadmapTool::ReadGeometriesFromFile.
// Loading the graph requires neither OMPL nor open-spite.

#include "spite_d/common/types.hpp"
#include "spite_d/planner/replanner.hpp"

#include <string>
#include <vector>

namespace spite_d {

struct RoadmapGraph {
  /// Vertex states indexed by VID. Rotation is a unit quaternion
  /// (x, y, z, w) to match the SE3 state it was sampled from.
  struct VertexState {
    Vec3 position{0, 0, 0};
    std::array<double, 4> quaternion{0, 0, 0, 1};
  };

  std::vector<VertexState> vertices;
  std::vector<Replanner::Edge> edges;
};

/// Text format "spite_d roadmap v1". Returns false on I/O failure.
bool WriteRoadmapGraph(const RoadmapGraph& graph, const std::string& path);

/// Returns false if the file is missing or malformed.
bool ReadRoadmapGraph(RoadmapGraph& graph, const std::string& path);

}  // namespace spite_d
