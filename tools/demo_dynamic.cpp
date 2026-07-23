// Terminal demo + ablation of the dynamic-validity pipeline (no ROS,
// no simulator). Loads a roadmap produced by build_roadmap and drives
// a simulated obstacle across the initial path in one or both modes:
//
//   baseline -- every frame: predict + rebuild obstacle geometry +
//               re-query the roadmap trees (frame-by-frame SPITE).
//   spans    -- SpanPipeline: geometry frozen per horizon; each frame
//               costs a conformance check, with rebuilds only at spawn,
//               scheduled refresh, or prediction violation.
//
// Both modes see the identical obstacle motion, so the printed summary
// is the paper's ablation in miniature: same classifications and
// replans, orders fewer expensive updates.
//
// Usage: demo_dynamic --roadmap DIR [--frames N] [--hz F]
//                     [--mode baseline|spans|both] [--trace]

#include "spite_d/dynamic_map/span_pipeline.hpp"
#include "spite_d/planner/replanner.hpp"
#include "spite_d/planner/roadmap_io.hpp"
#include "spite_d/spite/validity_server.hpp"
#include "spite_d/trajectory/predictor.hpp"

#include "DynamicRoadmapTool.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace spite_d;

constexpr double kSigmaGain = 1.0;

struct Options {
  std::string dir;
  int frames = 40;
  double hz = 10.0;
  std::string mode = "both";
  bool trace = false;
};

struct ModeResult {
  std::string name;
  size_t geometryRebuilds = 0;
  int replans = 0;
  std::vector<double> updateMs;
  std::vector<size_t> pathSizes;

  double Mean() const {
    double s = 0;
    for (double v : updateMs) s += v;
    return updateMs.empty() ? 0 : s / updateMs.size();
  }
  double Median() const {
    if (updateMs.empty()) return 0;
    std::vector<double> v = updateMs;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  }
  double Max() const {
    return updateMs.empty() ? 0
                            : *std::max_element(updateMs.begin(), updateMs.end());
  }
};

std::unique_ptr<DynamicRoadmapTool> LoadDrm(const std::string& dir) {
  auto drm = std::make_unique<DynamicRoadmapTool>();
  drm->SetUseUnderApprox(true);
  if (!drm->ReadGeometriesFromFile(dir + "/roadmap_geoms.txt")) return nullptr;
  return drm;
}

/// Runs one mode over the shared scenario; the obstacle is aimed at the
/// midpoint of the initial path and follows constant velocity exactly.
ModeResult RunMode(const std::string& name, const Options& opt,
                   const RoadmapGraph& graph, size_t start, size_t goal,
                   const Vec3& aimPoint) {
  ModeResult result;
  result.name = name;

  auto drm = LoadDrm(opt.dir);
  ValidityServer server(std::move(drm), {kSigmaGain});
  Replanner planner(graph.vertices.size(), graph.edges);
  ConstantVelocityPredictor predictor(/*stdGrowthRate=*/0.1);

  SpanPipeline::Params spanParams;
  spanParams.span.sigmaGain = kSigmaGain;
  spanParams.horizon = 2.0;
  spanParams.dt = 0.2;
  SpanPipeline pipeline(predictor, server, spanParams);
  const bool useSpans = name == "spans";

  const auto isValid = [&server](size_t a, size_t b) {
    return server.GetEdgeValidity(a, b) != ValidityServer::Validity::INVALID;
  };

  TrackedObstacle track;
  track.id = 1;
  track.velocity = {0.0, 1.2, 0.0};
  track.halfExtents = {0.4, 0.4, 0.4};
  track.positionStd = {0.05, 0.05, 0.05};

  std::vector<size_t> path = planner.Plan(start, goal, isValid);

  if (opt.trace)
    std::printf("[%s]\n%5s %12s %7s %7s %9s %10s\n", name.c_str(), "t[s]",
                "obstacle y", "gray", "red", "replanned", "update[ms]");

  for (int f = 0; f < opt.frames; ++f) {
    const double t = f / opt.hz;
    track.stamp = t;
    track.pose.translation = {aimPoint[0], (aimPoint[1] - 1.5) + 1.2 * t,
                              aimPoint[2]};

    const auto t0 = std::chrono::steady_clock::now();
    if (useSpans) {
      pipeline.Update({track});
    } else {
      server.Update({predictor.Predict(track, spanParams.horizon,
                                       spanParams.dt)});
    }
    result.updateMs.push_back(std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t0)
                                  .count());

    bool replanned = false;
    if (path.empty() || planner.PathBlocked(path, isValid)) {
      path = planner.Plan(start, goal, isValid);
      replanned = true;
      ++result.replans;
    }
    result.pathSizes.push_back(path.size());

    if (opt.trace) {
      size_t gray = 0, red = 0;
      for (const auto& e : graph.edges) {
        const auto v = server.GetEdgeValidity(e.src, e.tgt);
        if (v == ValidityServer::Validity::INVALID) ++red;
        else if (v == ValidityServer::Validity::UNKNOWN) ++gray;
      }
      std::printf("%5.1f %12.2f %7zu %7zu %9s %10.2f\n", t,
                  track.pose.translation[1], gray, red,
                  replanned ? "YES" : "-", result.updateMs.back());
    }
  }

  result.geometryRebuilds =
      useSpans ? pipeline.GetStats().rebuilds : size_t(opt.frames);
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--roadmap") && ++i < argc) opt.dir = argv[i];
    else if (!std::strcmp(argv[i], "--frames") && ++i < argc) opt.frames = std::atoi(argv[i]);
    else if (!std::strcmp(argv[i], "--hz") && ++i < argc) opt.hz = std::atof(argv[i]);
    else if (!std::strcmp(argv[i], "--mode") && ++i < argc) opt.mode = argv[i];
    else if (!std::strcmp(argv[i], "--trace")) opt.trace = true;
  }
  if (opt.dir.empty()) {
    std::fprintf(stderr,
                 "usage: demo_dynamic --roadmap DIR [--frames N] [--hz F] "
                 "[--mode baseline|spans|both] [--trace]\n");
    return 1;
  }

  RoadmapGraph graph;
  if (!ReadRoadmapGraph(graph, opt.dir + "/roadmap_graph.txt")) {
    std::fprintf(stderr, "cannot read %s/roadmap_graph.txt\n", opt.dir.c_str());
    return 1;
  }
  std::printf("roadmap: %zu vertices, %zu edges\n", graph.vertices.size(),
              graph.edges.size());

  // Start/goal near opposite workspace corners; aim point = midpoint of
  // the obstacle-free initial path (computed once so both modes share it).
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

  Vec3 aimPoint;
  {
    Replanner planner(graph.vertices.size(), graph.edges);
    const auto path =
        planner.Plan(start, goal, [](size_t, size_t) { return true; });
    if (path.empty()) {
      std::fprintf(stderr, "no initial path -- roadmap too sparse?\n");
      return 1;
    }
    aimPoint = graph.vertices[path[path.size() / 2]].position;
  }
  std::printf("query: %zu -> %zu, obstacle aimed at (%.1f, %.1f, %.1f)\n\n",
              start, goal, aimPoint[0], aimPoint[1], aimPoint[2]);

  std::vector<ModeResult> results;
  if (opt.mode == "baseline" || opt.mode == "both")
    results.push_back(RunMode("baseline", opt, graph, start, goal, aimPoint));
  if (opt.mode == "spans" || opt.mode == "both")
    results.push_back(RunMode("spans", opt, graph, start, goal, aimPoint));

  std::printf("%-9s %7s %14s %8s %26s\n", "mode", "frames", "geom-rebuilds",
              "replans", "update ms mean/med/max");
  for (const auto& r : results)
    std::printf("%-9s %7d %14zu %8d %12.3f /%7.3f /%7.3f\n", r.name.c_str(),
                opt.frames, r.geometryRebuilds, r.replans, r.Mean(),
                r.Median(), r.Max());

  if (results.size() == 2) {
    const auto& base = results[0];
    const auto& spans = results[1];
    if (spans.Mean() > 0)
      std::printf("\nspan speedup: %.1fx mean update (conforming frames are "
                  "~free: median %.4f ms), %zu -> %zu expensive geometry "
                  "passes\n",
                  base.Mean() / spans.Mean(), spans.Median(),
                  base.geometryRebuilds, spans.geometryRebuilds);
    if (base.pathSizes == spans.pathSizes && base.replans == spans.replans)
      std::printf("identical replan behavior in both modes "
                  "(same path sizes every frame)\n");
    else
      std::printf("NOTE: modes diverged in replan behavior "
                  "(%d vs %d replans) -- expected only from span "
                  "conservatism at rebuild boundaries\n",
                  base.replans, spans.replans);
  }
  return 0;
}
