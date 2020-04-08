#ifndef AI_SERVER_PLANNER_DETAIL_GEOMETRY_HELPER_H
#define AI_SERVER_PLANNER_DETAIL_GEOMETRY_HELPER_H

#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <Eigen/Core>

#include "ai_server/util/math/geometry_traits.h"

namespace ai_server::planner::detail {

// 障害物を包括するbox
using envelope_type = boost::geometry::model::box<Eigen::Vector2d>;

/// @brief ジオメトリに対する包括領域を返す
/// @param g ジオメトリ
template <class Geometry>
inline auto to_envelope(const Geometry& g) {
  return boost::geometry::return_envelope<envelope_type>(g);
}

/// @brief ジオメトリに対してマージンを持った包括領域を返す
/// @param g ジオメトリ
/// @param margin マージン
template <class Geometry>
inline auto to_envelope(const Geometry& g, double margin) {
  // 図形を包括するエリア
  auto env = boost::geometry::return_envelope<envelope_type>(g);
  // マージン分拡大
  env.min_corner() -= margin * Eigen::Vector2d::Ones();
  env.max_corner() += margin * Eigen::Vector2d::Ones();
  return env;
}
} // namespace ai_server::planner::detail

#endif
