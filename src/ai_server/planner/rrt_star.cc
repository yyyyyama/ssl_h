#include <cmath>
#include <random>
#include "ai_server/planner/rrt_star.h"
#include "ai_server/util/math/geometry.h"
#include "ai_server/util/math/to_vector.h"

// RTree用
namespace boost::geometry::index {
using node_ptr_t = std::shared_ptr<ai_server::planner::rrt_star::node>;

template <>
struct indexable<node_ptr_t> {
  using value_type  = node_ptr_t;
  using result_type = const Eigen::Vector2d&;
  result_type operator()(const value_type& v) const {
    return v->position;
  }
};
} // namespace boost::geometry::index

namespace ai_server {
namespace planner {

rrt_star::rrt_star(const model::world& world) : world_(world) {
  std::random_device rnd;
  mt_ = boost::random::mt19937{rnd()};
}

void rrt_star::set_obstacles(const std::vector<object>& obstacles) {
  std::vector<typename obstacles_tree_t::value_type> tmp;
  tmp.reserve(obstacles.size());
  std::transform(obstacles.begin(), obstacles.end(), std::back_inserter(tmp), [this](auto a) {
    return std::make_pair<box_t, object>({a.position - a.r * Eigen::Vector2d::Ones(),
                                          a.position + a.r * Eigen::Vector2d::Ones()},
                                         std::move(a));
  });

  // packing algorithmを使用して構築
  obstacles_ = obstacles_tree_t{std::make_move_iterator(tmp.begin()),
                                std::make_move_iterator(tmp.end())};
}

void rrt_star::set_lines(const Eigen::Vector2d& first_point,
                         const Eigen::Vector2d& second_point, double radius) {
  lines_.emplace_back(line(boost::geometry::model::segment(first_point, second_point), radius));
}

void rrt_star::search(const position_t& start, const position_t& goal,
                      const Eigen::Vector2d& max_pos, const Eigen::Vector2d& min_pos,
                      const int search_num, const double max_branch_length,
                      const double margin) {
  // フィールド情報の更新
  update_field();

  // スタート座標とゴール座標を変換
  const auto start_pos = util::math::position(start);
  const auto goal_pos  = util::math::position(goal);

  // 障害物の圏内から脱出する必要があるとき
  if (auto p = exit_position(start_pos, goal_pos, margin, max_branch_length)) {
    target_            = {p->x(), p->y(), goal.theta};
    trajectory_length_ = (*p - start_pos).norm();
    return;
  }

  // 目的地周辺のとき
  if ((start_pos - goal_pos).norm() < 80.0) {
    target_            = goal;
    trajectory_length_ = (goal_pos - start_pos).norm();
    return;
  }

  // === rrt star ===

  // 有効なエリア
  box_t area;
  boost::geometry::intersection(game_area_, box_t{min_pos, max_pos}, area);
  // 探索木(nodeの集まり)
  tree_t tree;

  // 初期ノード追加
  tree.insert(std::make_shared<node>(start_pos, 0.0, nullptr));

  // 基準半径
  const double r_a = 2000 / std::sqrt(std::log10(2.0) / 2.0);

  for (int c = 0; c < search_num; ++c) {
    // 新しいノード
    const auto new_node = make_node(goal_pos, margin, max_branch_length, area, tree);

    // 近傍点リストを作る円の半径
    const double r = r_a * std::sqrt(std::log10(c + 1) / (c + 1));

    // 先に大まかに絞り込むための範囲
    const box_t around{new_node->position - r * Eigen::Vector2d::Ones(),
                       new_node->position + r * Eigen::Vector2d::Ones()};

    // 近傍点リスト
    std::vector<std::weak_ptr<node>> list;
    // 半径と障害物でフィルタリング
    tree.query(boost::geometry::index::within(around) &&
                   boost::geometry::index::satisfies(
                       [this, &np = new_node->position, r, margin](const auto& a) {
                         if ((a->position - np).norm() < r) {
                           const line_t line{a->position, np};
                           return !is_obstructed(line, margin) && !in_penalty(line) &&
                                  !is_lined(line, margin);
                         }
                         return false;
                       }),
               std::back_inserter(list));

    // これまでの道と新しいノードからの道を比較してコスト低ならノード再接続
    if (!list.empty()) {
      const auto min = *std::min_element(
          list.begin(), list.end(), [& np = new_node->position](const auto& a, const auto& b) {
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

  // 目的地を探す。
  const auto& p = [this, &nearest_node, &start_pos, &goal_pos, margin, max_branch_length]() {
    // 目的地とその一つ手前のノード間の距離を代入
    trajectory_length_ = ((*nearest_node)->position - goal_pos).norm();

    // スムージングした後の値を返す。
    for (std::weak_ptr<node> n = *nearest_node; n.lock()->parent.lock(); n = n.lock()->parent) {
      const auto& p = n.lock()->position;
      priority_points_.push(p);

      // それぞれのノード間の距離を積分
      trajectory_length_ += (p - n.lock()->parent.lock()->position).norm();

      const line_t line{start_pos, p};

      // スタート地点とノード間に障害物が無くなったとき
      if (!is_obstructed(line, margin) && !in_penalty(line) && !is_lined(line, margin)) {
        // スタート地点とノード間の距離を加算
        trajectory_length_ += (p - start_pos).norm();

        return n.lock()->position;
      }
    }
    // start_posを指定し続ける
    trajectory_length_ = 0.0;
    return start_pos;
  }();

  target_ = {p.x(), p.y(), goal.theta};
}

void rrt_star::update_field() {
  // フォールドのマージン
  constexpr double f_margin = 200.0;

  const auto field = world_.field();

  const double p_length     = field.penalty_length();
  const double p_half_width = field.penalty_width() / 2.0;

  game_area_ = {{field.x_min() - f_margin, field.y_min() - f_margin},
                {field.x_max() + f_margin, field.y_max() + f_margin}};

  my_penalty_area_ = {{field.x_min(), -p_half_width}, {field.x_min() + p_length, p_half_width}};

  enemy_penalty_area_ = {{field.x_max() - p_length, -p_half_width},
                         {field.x_max(), p_half_width}};
}

std::optional<Eigen::Vector2d> rrt_star::exit_position(const Eigen::Vector2d& start,
                                                       const Eigen::Vector2d& goal,
                                                       double margin, double d) const {
  //線分の範囲内のとき
  if (is_lined(start, margin)) {
    for (const auto& tmp : lines_) {
      if (boost::geometry::distance(start, tmp.center_line) < margin + tmp.r) {
        //線分を作る2点
        const Eigen::Vector2d p1 = std::get<0>(tmp.center_line);
        const Eigen::Vector2d p2 = std::get<1>(tmp.center_line);

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
    }
  }

  // 避けきれない障害物を調べる
  if (!obstacles_.empty()) {
    // パフォーマンス向上のため，調べる範囲を限定する
    const box_t area{start - margin * Eigen::Vector2d::Ones(),
                     start + margin * Eigen::Vector2d::Ones()};

    // 実際に障害物に当たっているかを調べる
    // 近い障害物ほど優先度高めに
    for (auto itr =
             obstacles_.qbegin(boost::geometry::index::intersects(area) &&
                               boost::geometry::index::nearest(start, obstacles_.size()));
         itr != obstacles_.qend(); ++itr) {
      const auto& obj = std::get<1>(*itr);

      // 避ける範囲内に開始地点が存在するか
      if ((obj.position - start).norm() < margin + obj.r) {
        // そのままゴールに向かっても障害物に近づかない時
        if ((goal - start).dot(obj.position - start) < 0.0) {
          return start + d * (goal - start).normalized();
        }

        // 回避点を探す
        const auto [p1, p2] = util::math::calc_isosceles_vertexes(obj.position, start, d);
        // コースの距離がより短い方を選ぶ
        return std::min(p1, p2, [&goal](const auto& a, const auto& b) {
          return (a - goal).norm() < (b - goal).norm();
        });
      }
    }
  }

  // field外かペナルティエリア内のとき
  if (in_penalty(start) || !boost::geometry::within(start, game_area_)) {
    return Eigen::Vector2d::Zero();
  }

  // 障害物から離れる必要がない
  return std::nullopt;
}

std::shared_ptr<rrt_star::node> rrt_star::make_node(const Eigen::Vector2d& goal, double margin,
                                                    double max_branch_length, const box_t& area,
                                                    const tree_t& tree) {
  // ランダム点の分布
  constexpr double rand_margin = 1000.0;
  const boost::random::uniform_real_distribution<> rand_x{area.min_corner().x() - rand_margin,
                                                          area.max_corner().x() + rand_margin};
  const boost::random::uniform_real_distribution<> rand_y{area.min_corner().y() - rand_margin,
                                                          area.max_corner().y() + rand_margin};

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

      if (!is_obstructed(new_p, margin) && !in_penalty(new_p) && !is_lined(new_p, margin) &&
          boost::geometry::within(new_p, area)) {
        return std::make_shared<node>(
            new_p, (*nearest_node)->cost + (new_p - (*nearest_node)->position).norm(),
            *nearest_node);
      }
    }
  }
}

rrt_star::node::node(const Eigen::Vector2d& pos, double c, const std::shared_ptr<node>& ptr)
    : position(pos), cost(c), parent(ptr) {}

rrt_star::object::object(const Eigen::Vector2d& pos, double radius)
    : position(pos), r(radius) {}

rrt_star::line::line(const line_t& center, double radius) : center_line(center), r(radius) {}

} // namespace planner
} // namespace ai_server
