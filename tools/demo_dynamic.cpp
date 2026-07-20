// Terminal demo of the baseline dynamic-validity pipeline (no ROS, no
// simulator). Loads a roadmap produced by build_roadmap, then drives a
// simulated obstacle across the workspace. Every "sensor frame":
//
//   tracked obstacle -> constant-velocity prediction over the horizon
//   -> ValidityServer (RGG classification, +/-k*sigma geometry)
//   -> blocked-edge counts and replanning between two far-apart vertices.
//
// Intended for quick demos: shows Red/Green/Gray evolving as the
// obstacle sweeps through, the path breaking and healing, and per-frame
// update timings.
//
// Usage: demo_dynamic --roadmap DIR [--frames N] [--hz F]
//        (DIR must contain roadmap_graph.txt / roadmap_geoms.txt)

#include "spite_d/planner/replanner.hpp"
#include "spite_d/planner/roadmap_io.hpp"
#include "spite_d/spite/validity_server.hpp"
#include "spite_d/trajectory/predictor.hpp"

#include "DynamicRoadmapTool.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

int main(int argc, char** argv) {
  using namespace spite_d;

  std::string dir;
  int frames = 40;
  double hz = 10.0;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--roadmap") && ++i < argc) dir = argv[i];
    else if (!std::strcmp(argv[i], "--frames") && ++i < argc) frames = std::atoi(argv[i]);
    else if (!std::strcmp(argv[i], "--hz") && ++i < argc) hz = std::atof(argv[i]);
  }
  if (dir.empty()) {
    std::fprintf(stderr, "usage: demo_dynamic --roadmap DIR [--frames N] [--hz F]\n");
    return 1;
  }

  // ---- Load the roadmap.
  RoadmapGraph graph;
  if (!ReadRoadmapGraph(graph, dir + "/roadmap_graph.txt")) {
    std::fprintf(stderr, "cannot read %s/roadmap_graph.txt\n", dir.c_str());
    return 1;
  }
  auto drm = std::make_unique<DynamicRoadmapTool>();
  drm->SetUseUnderApprox(true);
  if (!drm->ReadGeometriesFromFile(dir + "/roadmap_geoms.txt")) {
    std::fprintf(stderr, "cannot read %s/roadmap_geoms.txt\n", dir.c_str());
    return 1;
  }
  std::printf("roadmap: %zu vertices, %zu edges\n", graph.vertices.size(),
              graph.edges.size());

  ValidityServer server(std::move(drm), {/*sigmaGain=*/1.0});
  Replanner planner(graph.vertices.size(), graph.edges);

  // Start/goal: vertices nearest the two workspace corners.
  const auto nearest = [&](double x, double y, double z) {
    size_t best = 0;
    double bestD = 1e30;
    for (size_t v = 0; v < graph.vertices.size(); ++v) {
      const auto& p = graph.vertices[v].position;
      const double d = std::pow(p[0] - x, 2) + std::pow(p[1] - y, 2) +
                       std::pow(p[2] - z, 2);
      if (d < bestD) { bestD = d; best = v; }
    }
    return best;
  };
  const size_t start = nearest(0.5, 0.5, 0.5);
  const size_t goal = nearest(9.5, 9.5, 9.5);
  std::printf("query: vertex %zu -> vertex %zu\n\n", start, goal);

  ConstantVelocityPredictor predictor(/*stdGrowthRate=*/0.1);
  const auto isValid = [&server](size_t a, size_t b) {
    return server.GetEdgeValidity(a, b) != ValidityServer::Validity::INVALID;
  };

  // ---- Simulated obstacle: a 0.8m cube aimed straight at the midpoint
  // of the initial path, so the demo shows the path break and heal.
  std::vector<size_t> path = planner.Plan(start, goal, isValid);
  if (path.empty()) {
    std::fprintf(stderr, "no initial path -- roadmap too sparse?\n");
    return 1;
  }
  const auto& mid = graph.vertices[path[path.size() / 2]].position;

  TrackedObstacle track;
  track.id = 1;
  track.pose.translation = {mid[0], mid[1] - 1.5, mid[2]};
  track.velocity = {0.0, 1.2, 0.0};
  track.halfExtents = {0.4, 0.4, 0.4};
  track.positionStd = {0.05, 0.05, 0.05};
  std::printf("obstacle aimed at path midpoint (%.1f, %.1f, %.1f)\n\n",
              mid[0], mid[1], mid[2]);
  int replans = 0;
  std::printf("%5s %14s %7s %7s %7s %9s %10s\n", "t[s]", "obstacle y",
              "gray", "red", "path", "replanned", "update[ms]");

  for (int f = 0; f < frames; ++f) {
    const double t = f / hz;
    track.stamp = t;
    track.pose.translation[1] = (mid[1] - 1.5) + track.velocity[1] * t;

    const auto t0 = std::chrono::steady_clock::now();
    server.Update({predictor.Predict(track, /*horizon=*/2.0, /*dt=*/0.2)});
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0).count();

    size_t gray = 0, red = 0;
    for (const auto& e : graph.edges) {
      const auto v = server.GetEdgeValidity(e.src, e.tgt);
      if (v == ValidityServer::Validity::INVALID) ++red;
      else if (v == ValidityServer::Validity::UNKNOWN) ++gray;
    }

    if (f == 12 && std::getenv("DEMO_DEBUG")) {
      for (size_t i = 0; i + 1 < path.size(); ++i) {
        const auto& a = graph.vertices[path[i]].position;
        const auto& b = graph.vertices[path[i + 1]].position;
        std::printf("  edge %zu->%zu  (%.1f,%.1f,%.1f)->(%.1f,%.1f,%.1f)  v=%d\n",
                    path[i], path[i + 1], a[0], a[1], a[2], b[0], b[1], b[2],
                    int(server.GetEdgeValidity(path[i], path[i + 1])));
      }
    }

    bool replanned = false;
    if (path.empty() || planner.PathBlocked(path, isValid)) {
      path = planner.Plan(start, goal, isValid);
      replanned = true;
      ++replans;
    }

    std::printf("%5.1f %14.2f %7zu %7zu %7zu %9s %10.2f\n", t,
                track.pose.translation[1], gray, red, path.size(),
                replanned ? "YES" : "-", ms);
  }

  std::printf("\n%d replans over %d frames; final path %zu vertices.\n",
              replans, frames, path.size());
  return 0;
}
