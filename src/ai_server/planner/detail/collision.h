#ifndef AI_SERVER_PLANNER_DETAIL_COLLISION_H
#define AI_SERVER_PLANNER_DETAIL_COLLISION_H

#include <algorithm>
#include <variant>
#include <boost/geometry/algorithms/distance.hpp>

#include "geometry_helper.h"
#include "obstacle_tree.h"

namespace ai_server::planner::detail {

/// @brief 障害物に対する当たり判定を行い，結果を返す
/// @param g ジオメトリ
/// @param obstacle 障害物
/// @param margin 当たったとみなす距離のしきい値．
/// @return 当たっている障害物が含まれているときtrue.
template <class Geometry, class Obstacle>
inline auto is_collided(const Geometry& g, const Obstacle& o, double margin) {
  return boost::geometry::distance(g, o.geometry) < margin + o.margin;
}

/// @brief 障害物に対する当たり判定を行い，結果を返す
/// @param g ジオメトリ
/// @param obstacle 障害物
/// @param margin 当たったとみなす距離のしきい値．
/// @return 当たっている障害物が含まれているときtrue.
template <class Geometry, class... ObstacleTypes>
inline auto is_collided(const Geometry& g, const std::variant<ObstacleTypes...>& o,
                        double margin) {
  return std::visit([&g, margin](const auto& arg) { return is_collided(g, arg, margin); }, o);
}

/// @brief 障害物に対する当たり判定を行い，結果を返す
/// @param g ジオメトリ
/// @param obstacles 障害物
/// @param margin 当たったとみなす距離のしきい値．
/// @return 当たっている障害物が含まれているときtrue.
template <class Geometry, class... ObstacleTypes>
inline auto is_collided(const Geometry& g, const tree_type<ObstacleTypes...>& obstacles,
                        double margin) {
  // 先に範囲を限定する
  const auto env = to_envelope(g, margin);
  const auto itr = obstacles.qbegin(boost::geometry::index::intersects(env));

  return std::any_of(itr, obstacles.qend(), [margin, &g](const auto& a) {
    // 詳しく調べる
    return is_collided(g, std::get<1>(a), margin);
  });
}

/// @brief 衝突している障害物を抽出する
/// @param obstacles 障害物
/// @param g ジオメトリ
/// @param margin 当たったとみなす距離のしきい値．
/// @return 衝突している障害物を指すイテレータ
template <class Geometry, class... ObstacleTypes>
inline auto extract_collisions(const tree_type<ObstacleTypes...>& obstacles, const Geometry& g,
                               double margin) {
  // 先に範囲を限定する
  const auto env = to_envelope(g, margin);

  return obstacles.qbegin(boost::geometry::index::intersects(env) &&
                          boost::geometry::index::satisfies([margin, &g](const auto& a) {
                            return is_collided(g, std::get<1>(a), margin);
                          }));
}
} // namespace ai_server::planner::detail

#endif
