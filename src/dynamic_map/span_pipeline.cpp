#include "spite_d/dynamic_map/span_pipeline.hpp"

namespace spite_d {

SpanPipeline::SpanPipeline(const Predictor& predictor, ValidityServer& server,
                           Params params)
    : m_predictor(predictor), m_server(server), m_params(params) {}

void SpanPipeline::Update(const std::vector<TrackedObstacle>& tracks) {
  ++m_stats.frames;

  std::vector<PredictedTrajectory> rebuilds;
  for (const TrackedObstacle& track : tracks) {
    ++m_stats.observations;

    const auto it = m_spans.find(track.id);
    if (it != m_spans.end() && it->second.Conforms(track) &&
        !it->second.NeedsRefresh(track.stamp)) {
      ++m_stats.conforming;
      continue;  // Certified by the existing span; no geometry work.
    }

    PredictedTrajectory traj =
        m_predictor.Predict(track, m_params.horizon, m_params.dt);
    m_spans.insert_or_assign(track.id, Span(traj, m_params.span));
    rebuilds.push_back(std::move(traj));
    ++m_stats.rebuilds;
  }

  // Only rebuilt slots get marked dirty; ValidityServer::Update runs
  // the RGG pass for those alone (no-op reconciliation otherwise).
  m_server.Update(rebuilds);
}

void SpanPipeline::Forget(int32_t obstacleId) {
  m_spans.erase(obstacleId);
  m_server.Forget(obstacleId);
}

const Span* SpanPipeline::GetSpan(int32_t obstacleId) const {
  const auto it = m_spans.find(obstacleId);
  return it == m_spans.end() ? nullptr : &it->second;
}

}  // namespace spite_d
