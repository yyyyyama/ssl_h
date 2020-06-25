#include <cmath>
#include <limits>
#include <random>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include "ai_server/planner/detail/collision.h"
#include "ai_server/util/math/geometry.h"
#include "ai_server/util/math/to_vector.h"
#include "rrt_star.h"

// RTree用
namespace boost::geometry::index {
using node_ptr_t = std::shared_ptr<ai_server::planner::impl::rrt_star::node>;

template <>
struct indexable<node_ptr_t> {
  using value_type  = node_ptr_t;
  using result_type = const Eigen::Vector2d&;
  result_type operator()(const value_type& v) const {
    return v->position;
  }
};
} // namespace boost::geometry::index

namespace ai_server::planner::impl {

using double_limits = std::numeric_limits<double>;

rrt_star::rrt_star()
    : node_count_(10),
      max_branch_length_(300.0),
      area_({double_limits::lowest(), double_limits::lowest()},
            {double_limits::max(), double_limits::max()}) {
  std::random_device rnd;
  mt_ = boost::random::mt19937{rnd()};
}

void rrt_star::set_max_pos(const Eigen::Vector2d& max_p) {
  area_.max_corner() = max_p;
}

void rrt_star::set_min_pos(const Eigen::Vector2d& min_p) {
  area_.min_corner() = min_p;
}

void rrt_star::set_node_count(int count) {
  node_count_ = count;
}

void rrt_star::set_max_branch_length(double length) {
  max_branch_length_ = length;
}

rrt_star::result_type rrt_star::execute(const Eigen::Vector2d& start_pos,
                                        const Eigen::Vector2d& goal_pos,
                                        const obstacle_list& obs) {
  // 障害物取得
  const auto obstacles = obs.to_tree();

  // 障害物の圏内から脱出する必要があるとき
  if (auto p = exit_position(start_pos, goal_pos, max_branch_length_, obstacles)) {
    return std::make_pair(*p, (*p - start_pos).norm());
  }

  // 目的地周辺のとき
  if ((start_pos - goal_pos).norm() < 80.0) {
    return std::make_pair(goal_pos, (goal_pos - start_pos).norm());
  }

  // === rrt star ===

  // 探索木(nodeの集まり)
  tree_t tree;

  // 初期ノード追加
  tree.insert(std::make_shared<node>(start_pos, 0.0, nullptr));

  // 基準半径
  const double r_a = 2000 / std::sqrt(std::log10(2.0) / 2.0);

  for (int c = 0; c < node_count_; ++c) {
    // 新しいノード
    const auto new_node = make_node(goal_pos, max_branch_length_, obstacles, tree);

    // 近傍点リストを作る円の半径
    const double r = r_a * std::sqrt(std::log10(c + 1) / (c + 1));

    // 先に大まかに絞り込むための範囲
    const auto around = detail::to_envelope(new_node->position, r);

    // 近傍点リスト
    std::vector<std::weak_ptr<node>> list;
    // 半径と障害物でフィルタリング
    tree.query(boost::geometry::index::within(around) &&
                   boost::geometry::index::satisfies(
                       [&obstacles, &np = new_node->position, r](const auto& a) {
                         return (a->position - np).norm() < r &&
                                !detail::is_collided(line_t{a->position, np}, obstacles);
                       }),
               std::back_inserter(list));

    // これまでの道と新しいノードからの道を比較してコスト低ならノード再接続
    if (!list.empty()) {
      const auto min = *std::min_element(
          list.begin(), list.end(), [&np = new_node->position](const auto& a, const auto& b) {
            return a.lock()->cost + (np - a.lock()->position).norm() <
                   b.lock()->cost + (np - b.lock()->position).norm();
          });

      //コスト低ならノード再接続
      const auto min_cost =
          min.lock()->cost + (new_node->position - min.lock()->position).norm();
      if (min_cost < new_node->cost) {
        new_node->parent = min.lock();
        new_node->cost   = min_cost;
      }
    }

    // treeに追加
    tree.insert(new_node);

    // 他のノードから新たなノードに再接続
    for (auto& a : list) {
      const double cost = new_node->cost + (new_node->position - a.lock()->position).norm();
      if (cost < a.lock()->cost) {
        a.lock()->parent = new_node;
        a.lock()->cost   = cost;
      }
    }
  }

  // 目標位置に最も近いノード
  const auto nearest_node = tree.qbegin(boost::geometry::index::nearest(goal_pos, 1));

  double trajectory_length;

  // 目的地を探す。
  const auto& p = [this, &nearest_node, &start_pos, &goal_pos, &obstacles,
                   &trajectory_length]() {
    // 目的地とその一つ手前のノード間の距離を代入
    trajectory_length = ((*nearest_node)->position - goal_pos).norm();

    // スムージングした後の値を返す。
    for (std::weak_ptr<node> n = *nearest_node; n.lock()->parent.lock(); n = n.lock()->parent) {
      const auto& p = n.lock()->position;
      priority_points_.push(p);

      // それぞれのノード間の距離を積分
      trajectory_length += (p - n.lock()->parent.lock()->position).norm();

      // スタート地点とノード間に障害物が無くなったとき
      if (!detail::is_collided(line_t{start_pos, p}, obstacles)) {
        // スタート地点とノード間の距離を加算
        trajectory_length += (p - start_pos).norm();

        return n.lock()->position;
      }
    }
    // start_posを指定し続ける
    trajectory_length = 0.0;
    return start_pos;
  }();

  return std::make_pair(p, trajectory_length);
}

std::optional<Eigen::Vector2d> rrt_star::exit_position(
    const Eigen::Vector2d& start, const Eigen::Vector2d& goal, double d,
    const obstacle_list::tree_type& obstacles) const {
  const auto collided_itr = detail::extract_collisions(obstacles, start);

  // 衝突する障害物がない
  if (collided_itr == obstacles.qend()) {
    // field外のとき
    if (!boost::geometry::within(start, area_)) {
      return Eigen::Vector2d::Zero();
    }

    // 障害物から離れる必要がない
    return std::nullopt;
  }

  // 障害物との距離を取得する関数
  auto obstacle_distance = [&start](auto&& o) {
    return std::visit(
        [&start](auto&& arg) { return boost::geometry::distance(start, arg.geometry); }, o);
  };

  // 最も近い要素を探す
  const auto nearest = *std::min_element(
      collided_itr, obstacles.qend(), [&obstacle_distance](const auto& a, const auto& b) {
        return obstacle_distance(std::get<1>(a)) < obstacle_distance(std::get<1>(b));
      });

  const auto& obstacle = std::get<1>(nearest);

  //線分のとき
  if (const auto l = std::get_if<model::obstacle::segment>(&obstacle)) {
    //線分を作る2点
    const auto& [p1, p2] = l->geometry;

    //線分を表すベクトル
    const Eigen::Vector2d line_vec = p2 - p1;

    //線分の端の一点からstartへの方向のベクトル
    const Eigen::Vector2d p1_to_start_vec = start - p1;
    const Eigen::Vector2d p2_to_start_vec = start - p2;

    //線分に対して法線ベクトルが引けず、p1側にstartがある時
    if (line_vec.dot(p1_to_start_vec) < 0.0) {
      return start + d * p2_to_start_vec.normalized();

      //線分に対して法線ベクトルが引けず、p2側にstartがある時
    } else if (line_vec.dot(p2_to_start_vec) > 0.0) {
      return start + d * p1_to_start_vec.normalized();

      //線分に対して法線ベクトルが引けるとき
    } else {
      //通常の線分のとき
      if (line_vec.norm() != 0) {
        //線分を表すベクトルによって出来る直線上で、startに最も近い点
        const Eigen::Vector2d nearest_point =
            p1 + line_vec * (line_vec.dot(p1_to_start_vec) / std::pow(line_vec.norm(), 2));

        //単位法線ベクトル(ロボットの動く方向)
        const Eigen::Vector2d no_vec = (start - nearest_point).normalized();

        return start + d * no_vec;

        //線分が短すぎて点になったとき
      } else {
        //線分を表すベクトルによって出来る直線上で、startに最も近い点
        const Eigen::Vector2d nearest_point = p1;

        //単位法線ベクトル(ロボットの動く方向)
        const Eigen::Vector2d no_vec = (start - nearest_point).normalized();

        return start + d * no_vec;
      }
    }
  }
  // 円のとき
  else if (const auto point = std::get_if<model::obstacle::point>(&obstacle)) {
    const auto& obs_p = point->geometry;

    // そのままゴールに向かっても障害物に近づかない時
    if ((goal - start).dot(obs_p - start) < 0.0) {
      return start + d * (goal - start).normalized();
    }

    // 回避点を探す
    const auto [p1, p2] = util::math::calc_isosceles_vertexes(obs_p, start, d);
    // コースの距離がより短い方を選ぶ
    return std::min(p1, p2, [&goal](const auto& a, const auto& b) {
      return (a - goal).norm() < (b - goal).norm();
    });
  }
  // boxのとき
  else if (const auto box = std::get_if<model::obstacle::box>(&obstacle)) {
    // 近似Boxがフィールドの端に触れる
    if (!boost::geometry::within(std::get<0>(nearest), area_)) {
      // フィールドの中心へ
      return boost::geometry::return_centroid<point_t>(area_);
    }
    // 障害物の中心点
    const auto center = boost::geometry::return_centroid<point_t>(box->geometry);
    // boxの中心から離れる
    return start + d * (start - center).normalized();
  }

  // エラー
  return std::nullopt;
}

std::shared_ptr<rrt_star::node> rrt_star::make_node(const Eigen::Vector2d& goal,
                                                    double max_branch_length,
                                                    const obstacle_list::tree_type& obstacles,
                                                    const tree_t& tree) {
  // ランダム点の分布
  constexpr double rand_margin = 1000.0;
  const boost::random::uniform_real_distribution<> rand_x{area_.min_corner().x() - rand_margin,
                                                          area_.max_corner().x() + rand_margin};
  const boost::random::uniform_real_distribution<> rand_y{area_.min_corner().y() - rand_margin,
                                                          area_.max_corner().y() + rand_margin};

  // ある位置から一定距離をおいて新たな位置を作る
  auto to_new_p = [max_branch_length](const Eigen::Vector2d& p,
                                      const Eigen::Vector2d& sample_pos) -> Eigen::Vector2d {
    const auto l = std::min(max_branch_length, (sample_pos - p).norm());
    return p + l * (sample_pos - p).normalized();
  };

  bool goal_selected = false;
  bool cash_empty    = priority_points_.empty();

  for (;;) {
    // サンプル点
    const auto sample = [this, &goal_selected, &cash_empty, &goal, &rand_x, &rand_y]() {
      if (goal_selected) {
        if (cash_empty) {
          return Eigen::Vector2d{rand_x(mt_), rand_y(mt_)};
        }

        // 前回候補点になっていた場所を候補に
        const auto p = priority_points_.front();
        priority_points_.pop();
        cash_empty = priority_points_.empty();
        return p;
      }

      // ゴールを候補地点に(一回のみ)
      goal_selected = true;
      return goal;
    }();

    // 最も近い点
    const auto nearest_node = tree.qbegin(boost::geometry::index::nearest(sample, 1));

    // 同じ点だったら却下
    if (!boost::geometry::equals(sample, (*nearest_node)->position)) {
      // 最も近い点から一定距離を置いて点を打ち、コースに障害物がないことを確認
      const auto new_p = to_new_p((*nearest_node)->position, sample);

      if (!detail::is_collided(new_p, obstacles) && boost::geometry::within(new_p, area_)) {
        return std::make_shared<node>(
            new_p, (*nearest_node)->cost + (new_p - (*nearest_node)->position).norm(),
            *nearest_node);
      }
    }
  }
}

rrt_star::node::node(const Eigen::Vector2d& pos, double c, const std::shared_ptr<node>& ptr)
    : position(pos), cost(c), parent(ptr) {}
} // namespace ai_server::planner::impl
