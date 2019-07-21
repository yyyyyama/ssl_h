#ifndef AI_SERVER_PLANNER_RRT_STAR_H
#define AI_SERVER_PLANNER_RRT_STAR_H

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <vector>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/random.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ai_server/model/command.h"
#include "ai_server/model/world.h"
#include "ai_server/planner/base.h"

namespace ai_server {
namespace planner {
class rrt_star : public base {
  using double_limits = std::numeric_limits<double>;

public:
  rrt_star(const model::world& world);

  // 節点
  struct node {
    boost::geometry::model::d2::point_xy<double> position; // 座標
    double cost;                // 親ノードまでに必要なコスト
    std::weak_ptr<node> parent; // 親ノードを指すポインタ

    node(const Eigen::Vector2d& pos, double c, const std::shared_ptr<node>& ptr);
  };

  // 障害物
  struct object {
    Eigen::Vector2d position; // 座標
    double r;                 // 物体の半径

    object(const Eigen::Vector2d& pos, double radius);
  };

  /// @brief  障害物の任意指定
  /// @param  obstacles 障害物(struct object)のvector
  void set_obstacles(const std::vector<object>& obstacles);

  /// @brief  探索関数(木の生成)
  /// @param  start 初期位置
  /// @param  goal  目標位置
  /// @param  max_pos 探索最大座標
  /// @param  min_pos 探索最小座標
  /// @param  search_num  探索を行う回数
  /// @param  max_branch_length 伸ばす枝の最大距離
  /// @param  margin  避けるときのマージン
  void search(const position_t& start, const position_t& goal,
              const Eigen::Vector2d& max_pos = {double_limits::max(), double_limits::max()},
              const Eigen::Vector2d& min_pos = {double_limits::lowest(),
                                                double_limits::lowest()},
              const int search_num = 10, const double max_branch_length = 300.0,
              const double margin = 120.0);

private:
  using point_t = boost::geometry::model::d2::point_xy<double>;
  using box_t   = boost::geometry::model::box<point_t>;
  using line_t  = boost::geometry::model::segment<point_t>;
  using tree_t =
      boost::geometry::index::rtree<std::shared_ptr<node>, boost::geometry::index::rstar<20>>;
  // objectと，それを近似したboxで表現
  using obstacles_tree_t = boost::geometry::index::rtree<std::pair<box_t, object>,
                                                         boost::geometry::index::rstar<20>>;

  const model::world& world_;
  // サンプル取得時用の乱数生成器(処理速度が優れているため，boostのものを使用)
  mutable boost::random::mt19937 mt_;

  // フィールド
  box_t game_area_;
  // 敵のペナルティエリア
  box_t enemy_penalty_area_;
  // 自分のペナルティエリア
  box_t my_penalty_area_;
  // 障害物
  obstacles_tree_t obstacles_;
  // あるループで生成された最適なルート木，次ループで優先して探索
  std::queue<Eigen::Vector2d> priority_points_;

  /// @brief  フィールド情報を更新する
  void update_field();

  // point_tに変換
  point_t to_point(const Eigen::Vector2d& p) const;

  // line_tに変換
  line_t to_line(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;

  // Eigen::Vector2dに変換
  Eigen::Vector2d to_vector(const point_t& p) const;

  /// @brief  障害物が存在するか
  /// @param  geometry 図形
  /// @param  margin  避けるときのマージン
  template <class Geometry>
  auto is_obstructed(const Geometry& geometry, const double margin) const
      -> decltype(boost::geometry::distance(point_t(), geometry), bool()) {
    // 図形を包括するエリア
    auto area = boost::geometry::return_envelope<box_t>(geometry);
    // マージン分拡大
    area.min_corner().x(area.min_corner().x() - margin);
    area.min_corner().y(area.min_corner().y() - margin);
    area.max_corner().x(area.max_corner().x() + margin);
    area.max_corner().y(area.max_corner().y() + margin);

    // 先に範囲を限定する
    const auto itr = obstacles_.qbegin(boost::geometry::index::intersects(area));

    return std::any_of(itr, obstacles_.qend(), [this, margin, &geometry](const auto& a) {
      const auto& obj = std::get<1>(a);
      // 詳しく調べる
      return boost::geometry::distance(to_point(obj.position), geometry) < margin + obj.r;
    });
  }

  /// @brief  ペナルティエリア内か
  /// @param  geometry 図形
  /// @param  margin  避けるときのマージン
  template <class Geometry>
  auto in_penalty(const Geometry& geometry) const
      -> decltype(boost::geometry::distance(geometry, box_t()), bool()) {
    constexpr double margin = 150.0;
    return boost::geometry::distance(geometry, my_penalty_area_) < margin ||
           boost::geometry::distance(geometry, enemy_penalty_area_) < margin;
  }

  /// @brief  障害物から離れる必要がある時，移動先を求める
  /// @param  start   初期位置
  /// @param  goal    目標位置
  /// @param  margin  障害物までのマージン
  /// @param  d       初期位置から移動先までの距離
  std::optional<Eigen::Vector2d> exit_position(const Eigen::Vector2d& start,
                                               const Eigen::Vector2d& goal, double margin,
                                               double d) const;

  /// @brief  あるエリアの範囲内で新規のノードを作成する
  /// @param  goal               最終目的地
  /// @param  margin  避けるときのマージン
  /// @param  max_branch_length  ノード間長さの最大値
  ///  @param area               有効なエリア
  /// @param  tree               探索木
  std::shared_ptr<node> make_node(const Eigen::Vector2d& goal, double margin,
                                  double max_branch_length, const box_t& area,
                                  const tree_t& tree);
};
} // namespace planner
} // namespace ai_server

#endif // AI_SERVER_PLANNER_RRT_STAR_H
