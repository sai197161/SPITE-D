#include "spite_d/planner/roadmap_io.hpp"

#include <fstream>

namespace spite_d {

namespace {
constexpr const char* kHeader = "spite_d roadmap v1";
}

bool WriteRoadmapGraph(const RoadmapGraph& graph, const std::string& path) {
  std::ofstream out(path);
  if (!out) return false;

  out.precision(17);
  out << kHeader << "\n";
  out << "verts " << graph.vertices.size() << "\n";
  for (const auto& v : graph.vertices) {
    out << v.position[0] << " " << v.position[1] << " " << v.position[2];
    for (double q : v.quaternion) out << " " << q;
    out << "\n";
  }
  out << "edges " << graph.edges.size() << "\n";
  for (const auto& e : graph.edges)
    out << e.src << " " << e.tgt << " " << e.cost << "\n";
  return static_cast<bool>(out);
}

bool ReadRoadmapGraph(RoadmapGraph& graph, const std::string& path) {
  std::ifstream in(path);
  if (!in) return false;

  std::string header;
  std::getline(in, header);
  if (header != kHeader) return false;

  std::string token;
  size_t count = 0;

  if (!(in >> token >> count) || token != "verts") return false;
  graph.vertices.assign(count, {});
  for (auto& v : graph.vertices) {
    if (!(in >> v.position[0] >> v.position[1] >> v.position[2] >>
          v.quaternion[0] >> v.quaternion[1] >> v.quaternion[2] >>
          v.quaternion[3]))
      return false;
  }

  if (!(in >> token >> count) || token != "edges") return false;
  graph.edges.assign(count, {});
  for (auto& e : graph.edges) {
    if (!(in >> e.src >> e.tgt >> e.cost)) return false;
    if (e.src >= graph.vertices.size() || e.tgt >= graph.vertices.size())
      return false;
  }
  return true;
}

}  // namespace spite_d
