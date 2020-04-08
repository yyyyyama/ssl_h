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
/// @return 当たっている障害物が含まれているときtrue.
template <class Geometry, class Obstacle>
inline auto is_collided(const Geometry& g, const Obstacle& o) {
  return boost::geometry::distance(g, o.geometry) < o.margin;
}

/// @brief 障害物に対する当たり判定を行い，結果を返す
/// @param g ジオメトリ
/// @param obstacle 障害物
/// @return 当たっている障害物が含まれているときtrue.
template <class Geometry, class... ObstacleTypes>
inline auto is_collided(const Geometry& g, const std::variant<ObstacleTypes...>& o) {
  return std::visit([&g](const auto& arg) { return is_collided(g, arg); }, o);
}

/// @brief 障害物に対する当たり判定を行い，結果を返す
/// @param g ジオメトリ
/// @param obstacles 障害物
/// @return 当たっている障害物が含まれているときtrue.
template <class Geometry, class... ObstacleTypes>
inline auto is_collided(const Geometry& g, const tree_type<ObstacleTypes...>& obstacles) {
  // 先に範囲を限定する
  const auto env = to_envelope(g);
  const auto itr = obstacles.qbegin(boost::geometry::index::intersects(env));

  return std::any_of(itr, obstacles.qend(), [&g](const auto& a) {
    // 詳しく調べる
    return is_collided(g, std::get<1>(a));
  });
}

/// @brief 衝突している障害物を抽出する
/// @param obstacles 障害物
/// @param g ジオメトリ
/// @return 衝突している障害物を指すイテレータ
template <class Geometry, class... ObstacleTypes>
inline auto extract_collisions(const tree_type<ObstacleTypes...>& obstacles,
                               const Geometry& g) {
  // 先に範囲を限定する
  const auto env = to_envelope(g);

  return obstacles.qbegin(boost::geometry::index::intersects(env) &&
                          boost::geometry::index::satisfies(
                              [&g](const auto& a) { return is_collided(g, std::get<1>(a)); }));
}
} // namespace ai_server::planner::detail

#endif
