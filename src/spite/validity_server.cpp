#include "spite_d/spite/validity_server.hpp"

#include "DynamicRoadmapTool.h"
#include "Geometry/SimpleGeometries.h"
#include "OverApprox.h"

#include <algorithm>
#include <unordered_map>

namespace spite_d {

namespace {

using Point3d = DynamicRoadmapTool::Point3d;

/// World-frame corners of an OBB given center pose and (possibly
/// inflated) half extents.
void AppendCorners(const Pose3& pose, const Vec3& half,
                   open_spite::PointCloud3d& out) {
  for (int sx : {-1, 1})
    for (int sy : {-1, 1})
      for (int sz : {-1, 1}) {
        const Vec3 local{sx * half[0], sy * half[1], sz * half[2]};
        Vec3 world;
        for (int r = 0; r < 3; ++r)
          world[r] = pose.rotation[r][0] * local[0] +
                     pose.rotation[r][1] * local[1] +
                     pose.rotation[r][2] * local[2] + pose.translation[r];
        out.emplace_back(world[0], world[1], world[2]);
      }
}

ValidityServer::Validity FromTool(DynamicRoadmapTool::Validity v) {
  switch (v) {
    case DynamicRoadmapTool::Validity::VALID:
      return ValidityServer::Validity::VALID;
    case DynamicRoadmapTool::Validity::INVALID:
      return ValidityServer::Validity::INVALID;
    default:
      return ValidityServer::Validity::UNKNOWN;
  }
}

}  // namespace

struct ValidityServer::Impl {
  std::unique_ptr<DynamicRoadmapTool> drm;
  Params params;
  std::unordered_map<int32_t, size_t> trackToSlot;
  size_t nextSlot{0};
};

ValidityServer::ValidityServer(std::unique_ptr<DynamicRoadmapTool> drm,
                               Params params)
    : m_impl(new Impl{std::move(drm), params, {}, 0}) {}

ValidityServer::~ValidityServer() = default;

void ValidityServer::Update(
    const std::vector<PredictedTrajectory>& predictions) {
  auto& impl = *m_impl;

  for (const PredictedTrajectory& traj : predictions) {
    if (traj.poses.empty()) continue;

    auto [it, inserted] = impl.trackToSlot.try_emplace(traj.id, impl.nextSlot);
    if (inserted) ++impl.nextSlot;
    const size_t slot = it->second;

    // Over-approximation: OBB of the predicted OBB corners across the
    // horizon, each sample inflated by +k*sigma.
    open_spite::PointCloud3d cloud;
    cloud.reserve(traj.poses.size() * 8);
    for (size_t i = 0; i < traj.poses.size(); ++i) {
      Vec3 half = traj.halfExtents;
      if (i < traj.positionStd.size())
        for (int a = 0; a < 3; ++a)
          half[a] += impl.params.sigmaGain * traj.positionStd[i][a];
      AppendCorners(traj.poses[i], half, cloud);
    }
    OrientedBoundingBox obb;
    open_spite::GetBoundingBox(cloud, obb);
    impl.drm->UpdateObstacleOBB(slot, obb);

    // Under-approximation: the obstacle's inscribed sphere placed at
    // each predicted center, deflated by -k*sigma. Samples whose
    // deflated radius vanishes contribute nothing (no false REDs).
    const double baseRadius = *std::min_element(traj.halfExtents.begin(),
                                                traj.halfExtents.end());
    std::vector<DynamicRoadmapTool::Sphere> spheres;
    spheres.reserve(traj.poses.size());
    for (size_t i = 0; i < traj.poses.size(); ++i) {
      double sigma = 0.0;
      if (i < traj.positionStd.size())
        sigma = *std::max_element(traj.positionStd[i].begin(),
                                  traj.positionStd[i].end());
      const double radius = baseRadius - impl.params.sigmaGain * sigma;
      if (radius <= 0.0) continue;
      const auto& t = traj.poses[i].translation;
      spheres.push_back({Point3d(t[0], t[1], t[2]), radius});
    }
    impl.drm->UpdateObstacleSpheres(slot, spheres);
  }

  impl.drm->ShallowUpdate();
}

void ValidityServer::Forget(int32_t obstacleId) {
  auto& impl = *m_impl;
  auto it = impl.trackToSlot.find(obstacleId);
  if (it == impl.trackToSlot.end()) return;

  // Park the slot's geometry far outside any workspace so reconciliation
  // drops all of its contributions, re-validating anything only it was
  // blocking. (An AABB, not a default OBB: open-spite's OBB queries
  // assume corners exist and crash on a cornerless box.)
  constexpr double kFar = 1e9;
  impl.drm->UpdateObstacleAABB(
      it->second, DynamicRoadmapTool::Box3d(Point3d(kFar, kFar, kFar),
                                            Point3d(kFar + 1, kFar + 1, kFar + 1)));
  impl.drm->UpdateObstacleSpheres(it->second, {});
  impl.drm->ShallowUpdate();
  impl.trackToSlot.erase(it);
}

ValidityServer::Validity ValidityServer::GetVertexValidity(size_t vid) const {
  return FromTool(m_impl->drm->GetVertexValidity(vid));
}

ValidityServer::Validity ValidityServer::GetEdgeValidity(size_t src,
                                                         size_t tgt) const {
  return FromTool(m_impl->drm->GetEdgeValidity(src, tgt));
}

std::vector<std::pair<size_t, size_t>> ValidityServer::BlockedEdges() const {
  std::vector<std::pair<size_t, size_t>> blocked;
  for (const auto& [element, geoms] : m_impl->drm->GetElementMap()) {
    if (element.size() != 2) continue;
    if (FromTool(m_impl->drm->GetEdgeValidity(element[0], element[1])) !=
        Validity::VALID)
      blocked.emplace_back(element[0], element[1]);
  }
  return blocked;
}

}  // namespace spite_d
