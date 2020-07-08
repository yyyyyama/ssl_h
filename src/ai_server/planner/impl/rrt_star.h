#ifndef AI_SERVER_PLANNER_IMPL_RRT_STAR_H
#define AI_SERVER_PLANNER_IMPL_RRT_STAR_H

#include <memory>
#include <optional>
#include <queue>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/random.hpp>
#include <Eigen/Core>

#include "ai_server/util/math/geometry_traits.h"
#include "ai_server/planner/obstacle_list.h"

namespace ai_server::planner::impl {
class rrt_star {
public:
  // 節点
  struct node {
    Eigen::Vector2d position;   // 座標
    double cost;                // 親ノードまでに必要なコスト
    std::weak_ptr<node> parent; // 親ノードを指すポインタ

    node(const Eigen::Vector2d& pos, double c, const std::shared_ptr<node>& ptr);
  };

  using result_type = std::pair<Eigen::Vector2d, double>;

  rrt_star();

  /// @brief 移動可能領域を設定する
  /// @param min_p 移動可能領域の最大座標
  void set_max_pos(const Eigen::Vector2d& max_p);

  /// @brief 移動可能領域設定する
  /// @param max_p 移動可能領域の最小座標
  void set_min_pos(const Eigen::Vector2d& min_p);

  /// @brief ノードを作る回数を設定する
  /// @param count 設定値．
  void set_node_count(int count);

  /// @brief 伸ばす枝の最大距離を設定する
  /// @param length 設定値．
  void set_max_branch_length(double length);

  result_type execute(const Eigen::Vector2d& start_pos, const Eigen::Vector2d& goal_pos,
                      const obstacle_list& obs);

private:
  using point_t = Eigen::Vector2d;
  using box_t   = boost::geometry::model::box<point_t>;
  using line_t  = boost::geometry::model::segment<point_t>;
  using tree_t =
      boost::geometry::index::rtree<std::shared_ptr<node>, boost::geometry::index::rstar<20>>;

  // サンプル取得時用の乱数生成器(処理速度が優れているため，boostのものを使用)
  mutable boost::random::mt19937 mt_;

  // 探索を行う回数
  int node_count_;

  // 伸ばす枝の最大距離
  double max_branch_length_;

  // 移動可能領域
  detail::envelope_type area_;

  // あるループで生成された最適なルート木，次ループで優先して探索
  std::queue<Eigen::Vector2d> priority_points_;

  /// @brief  障害物から離れる必要がある時，移動先を求める
  /// @param  start   初期位置
  /// @param  goal    目標位置
  /// @param  d       初期位置から移動先までの距離
  //  @param  obstacles 障害物
  std::optional<Eigen::Vector2d> exit_position(const Eigen::Vector2d& start,
                                               const Eigen::Vector2d& goal, double d,
                                               const obstacle_list::tree_type& obstacles) const;

  /// @brief  あるエリアの範囲内で新規のノードを作成する
  /// @param  goal               最終目的地
  /// @param  max_branch_length  ノード間長さの最大値
  /// @param  obstacles          障害物
  /// @param  tree               探索木
  std::shared_ptr<node> make_node(const Eigen::Vector2d& goal, double max_branch_length,
                                  const obstacle_list::tree_type& obstacles,
                                  const tree_t& tree);
};
} // namespace ai_server::planner::impl

#endif // AI_SERVER_PLANNER_RRT_STAR_H
