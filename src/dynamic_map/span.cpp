#include "spite_d/dynamic_map/span.hpp"

#include <algorithm>
#include <cmath>

namespace spite_d {

Span::Span(PredictedTrajectory trajectory, SpanParams params)
    : m_traj(std::move(trajectory)), m_params(params) {}

std::optional<Vec3> Span::PredictedPosition(double t) const {
  const auto& stamps = m_traj.stamps;
  if (stamps.empty()) return std::nullopt;

  if (t <= stamps.front()) return m_traj.poses.front().translation;
  if (t >= stamps.back()) return m_traj.poses.back().translation;

  const auto upper = std::upper_bound(stamps.begin(), stamps.end(), t);
  const size_t hi = size_t(upper - stamps.begin());
  const size_t lo = hi - 1;
  const double span = stamps[hi] - stamps[lo];
  const double alpha = span > 0.0 ? (t - stamps[lo]) / span : 0.0;

  Vec3 p;
  for (int a = 0; a < 3; ++a)
    p[a] = (1.0 - alpha) * m_traj.poses[lo].translation[a] +
           alpha * m_traj.poses[hi].translation[a];
  return p;
}

Vec3 Span::EnvelopeHalfWidth(double t) const {
  // Sigma at the nearest sample at-or-after t (conservative: sigma is
  // non-decreasing in lookahead for any sane predictor).
  Vec3 sigma{0, 0, 0};
  if (!m_traj.positionStd.empty()) {
    const auto upper =
        std::lower_bound(m_traj.stamps.begin(), m_traj.stamps.end(), t);
    size_t idx = size_t(upper - m_traj.stamps.begin());
    idx = std::min(idx, m_traj.positionStd.size() - 1);
    sigma = m_traj.positionStd[idx];
  }
  Vec3 half;
  for (int a = 0; a < 3; ++a)
    half[a] = m_traj.halfExtents[a] + m_params.sigmaGain * sigma[a] +
              m_params.slack;
  return half;
}

bool Span::Conforms(const TrackedObstacle& observation) const {
  if (m_traj.stamps.empty()) return false;
  if (observation.stamp > EndTime()) return false;  // Exhausted.

  const auto predicted = PredictedPosition(observation.stamp);
  if (!predicted) return false;

  const Vec3 half = EnvelopeHalfWidth(observation.stamp);
  for (int a = 0; a < 3; ++a) {
    if (std::abs(observation.pose.translation[a] - (*predicted)[a]) > half[a])
      return false;
  }
  return true;
}

bool Span::NeedsRefresh(double t) const {
  if (m_traj.stamps.empty()) return true;
  const double horizon = EndTime() - StartTime();
  if (horizon <= 0.0) return true;
  return (t - StartTime()) >= m_params.refreshFraction * horizon;
}

}  // namespace spite_d
