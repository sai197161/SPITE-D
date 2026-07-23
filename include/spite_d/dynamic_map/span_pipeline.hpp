#pragma once

// SpanPipeline: the span-based amortization loop.
//
// Baseline (ValidityServer alone): every frame, every obstacle's
// prediction is rebuilt and its SPITE geometry re-queried against the
// roadmap trees.
//
// Span mode (this class): per obstacle, a Span freezes the prediction.
// Each frame costs only a conformance check (a few comparisons); the
// expensive predict + geometry + tree-query path runs ONLY when
//   (a) the obstacle violates its span (prediction wrong),
//   (b) the span nears exhaustion (refreshFraction of horizon spent),
//   (c) the obstacle is new.
// Because ValidityServer only marks rebuilt slots dirty, conforming
// obstacles are entirely skipped by the RGG update -- that skip is the
// measured contribution, reported via Stats.
//
// Soundness: Green/Red labels were certified against the span's
// inflated envelope; conformance guarantees the obstacle is still
// inside it, so the labels remain trustworthy between rebuilds.
//
// The conformance SpanParams.sigmaGain should match the
// ValidityServer's geometry sigmaGain: the envelope checked is then
// exactly the envelope the geometry was built from.

#include "spite_d/common/types.hpp"
#include "spite_d/dynamic_map/span.hpp"
#include "spite_d/spite/validity_server.hpp"
#include "spite_d/trajectory/predictor.hpp"

#include <cstdint>
#include <map>
#include <vector>

namespace spite_d {

class SpanPipeline {
 public:
  struct Params {
    SpanParams span;
    double horizon{2.0};
    double dt{0.2};
  };

  struct Stats {
    size_t frames{0};       ///< Update() calls.
    size_t observations{0}; ///< Obstacle observations processed.
    size_t rebuilds{0};     ///< Spans (re)built = expensive path taken.
    size_t conforming{0};   ///< Observations absorbed by an existing span.
  };

  /// Borrows the predictor and validity server (caller owns both).
  SpanPipeline(const Predictor& predictor, ValidityServer& server,
               Params params);

  /// Ingest one frame of tracked obstacles. Only violating / expiring /
  /// new tracks trigger prediction + geometry rebuild; the rest are
  /// certified by conformance alone.
  void Update(const std::vector<TrackedObstacle>& tracks);

  /// Drop a lost track and re-validate what it blocked.
  void Forget(int32_t obstacleId);

  const Stats& GetStats() const { return m_stats; }
  const Span* GetSpan(int32_t obstacleId) const;

 private:
  const Predictor& m_predictor;
  ValidityServer& m_server;
  Params m_params;
  std::map<int32_t, Span> m_spans;
  Stats m_stats;
};

}  // namespace spite_d
