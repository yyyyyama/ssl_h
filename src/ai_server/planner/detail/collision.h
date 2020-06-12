#ifndef AI_SERVER_PLANNER_DETAIL_COLLISION_H
#define AI_SERVER_PLANNER_DETAIL_COLLISION_H

#include <algorithm>
#include <cmath>
#include <variant>
#include <type_traits>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <Eigen/Core>

#include "ai_server/model/obstacle/point.h"
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

/// @brief 障害物にぶつかるときの長さを計算する
/// @param first rayの長さを小さい順に並べたリストの最初の要素
/// @param last  rayの長さを小さい順に並べたリストの末尾の次の要素
/// @param start 開始地点
/// @param dir rayを伸ばす方向
/// @param o 障害物
/// @return 障害物にぶつかる最小長さの要素
template <class Iterator, class Obstacle>
inline auto find_collided_length(Iterator first, Iterator last, const Eigen::Vector2d& start,
                                 const Eigen::Vector2d& dir, const Obstacle& o) {
  static_assert(std::is_floating_point_v<typename std::iterator_traits<Iterator>::value_type>);

  using ray_type = boost::geometry::model::segment<Eigen::Vector2d>;

  return std::partition_point(first, last, [&start, &dir, &o](auto l) {
    return !is_collided(ray_type{start, start + l * dir}, o);
  });
}

/// @brief 障害物にぶつかるときの長さを計算する
/// @param first rayの長さを小さい順に並べたリストの最初の要素
/// @param last  rayの長さを小さい順に並べたリストの末尾の次の要素
/// @param start 開始地点
/// @param dir rayを伸ばす方向
/// @param o 障害物
/// @return 障害物にぶつかる最小長さの要素
template <class Iterator>
inline auto find_collided_length(Iterator first, Iterator last, const Eigen::Vector2d& start,
                                 const Eigen::Vector2d& dir, const model::obstacle::point& o) {
  static_assert(std::is_floating_point_v<typename std::iterator_traits<Iterator>::value_type>);

  using ray_type = boost::geometry::model::segment<Eigen::Vector2d>;

  // 要素が空のとき
  if (first == last) return last;

  // 最大まで伸ばしたray
  const ray_type ray{start, start + *std::prev(last, 1) * dir};

  // 衝突するとき
  if (is_collided(ray, o)) {
    // スタートから障害物中心まで
    const Eigen::Vector2d to_obstacle = o.geometry - start;
    const double dot                  = to_obstacle.dot(dir);

    const double squared_height = (to_obstacle - dot * dir).squaredNorm();
    const double squared_hypot  = o.margin * o.margin;
    // 正確な衝突距離
    const double length = dot - std::sqrt(squared_hypot - squared_height);

    // 衝突距離以上の値が出てくる場所を探す
    return std::lower_bound(first, last, length);
  }

  // 衝突しないとき
  return last;
}

/// @brief 障害物にぶつかるときの長さを計算する
/// @param first rayの長さを小さい順に並べたリストの最初の要素
/// @param last  rayの長さを小さい順に並べたリストの末尾の次の要素
/// @param start 開始地点
/// @param dir rayを伸ばす方向
/// @param o 障害物
/// @return 障害物にぶつかる最小長さの要素
template <class Iterator, class... ObstacleTypes>
inline auto find_collided_length(Iterator first, Iterator last, const Eigen::Vector2d& start,
                                 const Eigen::Vector2d& dir,
                                 const std::variant<ObstacleTypes...>& o) {
  static_assert(std::is_floating_point_v<typename std::iterator_traits<Iterator>::value_type>);
  return std::visit(
      [&first, &last, &start, &dir](const auto& arg) {
        return find_collided_length(first, last, start, dir, arg);
      },
      o);
}

/// @brief 障害物にぶつかるときの長さを計算する
/// @param first rayの長さを小さい順に並べたリストの最初の要素
/// @param last  rayの長さを小さい順に並べたリストの末尾の次の要素
/// @param start 開始地点
/// @param dir rayを伸ばす方向
/// @param obstacles 障害物
/// @return 当たっている障害物が含まれているときtrue.
template <class Iterator, class... ObstacleTypes>
inline auto find_collided_length(Iterator first, Iterator last, const Eigen::Vector2d& start,
                                 const Eigen::Vector2d& dir,
                                 const tree_type<ObstacleTypes...>& obstacles) {
  static_assert(std::is_floating_point_v<typename std::iterator_traits<Iterator>::value_type>);

  using ray_type = boost::geometry::model::segment<Eigen::Vector2d>;

  // 要素が空のとき
  if (first == last) return last;

  // 最大まで伸ばしたray
  const ray_type ray{start, start + *std::prev(last, 1) * dir};

  // 先に範囲を限定する
  const auto env     = to_envelope(ray);
  const auto obs_itr = obstacles.qbegin(boost::geometry::index::intersects(env));

  for (auto itr = obs_itr; itr != obstacles.qend(); ++itr) {
    // 調べつつ探索範囲を狭めていく
    last = find_collided_length(first, last, start, dir, std::get<1>(*itr));

    // 探索できる要素が無い
    if (last == first) break;
  }

  return last;
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
