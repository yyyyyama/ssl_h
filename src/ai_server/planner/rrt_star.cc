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
  using result_type = const boost::geometry::model::d2::point_xy<double>&;
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
    return std::make_pair<box_t, object>({to_point(a.position - a.r * Eigen::Vector2d::Ones()),
                                          to_point(a.position + a.r * Eigen::Vector2d::Ones())},
                                         std::move(a));
  });

  // packing algorithmを使用して構築
  obstacles_ = obstacles_tree_t{std::make_move_iterator(tmp.begin()),
                                std::make_move_iterator(tmp.end())};
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
    target_ = {p->x(), p->y(), goal.theta};
    return;
  }

  // 目的地周辺のとき
  if ((start_pos - goal_pos).norm() < 200.0) {
    target_ = goal;
    return;
  }

  // === rrt star ===

  // 有効なエリア
  box_t area;
  boost::geometry::intersection(game_area_, box_t{to_point(min_pos), to_point(max_pos)}, area);
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
    const box_t around{to_point(to_vector(new_node->position) - r * Eigen::Vector2d::Ones()),
                       to_point(to_vector(new_node->position) + r * Eigen::Vector2d::Ones())};

    // 近傍点リスト
    std::vector<std::weak_ptr<node>> list;
    // 半径と障害物でフィルタリング
    tree.query(boost::geometry::index::within(around) &&
                   boost::geometry::index::satisfies(
                       [this, &np = new_node->position, r, margin](const auto& a) {
                         if (boost::geometry::distance(a->position, np) < r) {
                           const line_t line{a->position, np};
                           return !is_obstructed(line, margin) && !in_penalty(line);
                         }
                         return false;
                       }),
               std::back_inserter(list));

    // これまでの道と新しいノードからの道を比較してコスト低ならノード再接続
    if (!list.empty()) {
      const auto min = *std::min_element(
          list.begin(), list.end(), [& np = new_node->position](const auto& a, const auto& b) {
            return a.lock()->cost + boost::geometry::distance(np, a.lock()->position) <
                   b.lock()->cost + boost::geometry::distance(np, b.lock()->position);
          });

      //コスト低ならノード再接続
      const auto min_cost = min.lock()->cost +
                            boost::geometry::distance(new_node->position, min.lock()->position);
      if (min_cost < new_node->cost) {
        new_node->parent = min.lock();
        new_node->cost   = min_cost;
      }
    }

    // treeに追加
    tree.insert(new_node);

    // 他のノードから新たなノードに再接続
    for (auto& a : list) {
      const double cost =
          new_node->cost + boost::geometry::distance(new_node->position, a.lock()->position);
      if (cost < a.lock()->cost) {
        a.lock()->parent = new_node;
        a.lock()->cost   = cost;
      }
    }
  }

  // 目標位置に最も近いノード
  const auto nearest_node = tree.qbegin(boost::geometry::index::nearest(to_point(goal_pos), 1));

  // 目的地を探す。
  const auto& p = [this, &nearest_node, &start_pos, &goal_pos, margin, max_branch_length]() {
    // スムージングした後の値を返す。
    for (std::weak_ptr<node> n = *nearest_node; n.lock()->parent.lock(); n = n.lock()->parent) {
      const auto& p = n.lock()->position;
      priority_points_.push(to_vector(p));

      const line_t line{to_point(start_pos), p};
      if (!is_obstructed(line, margin) && !in_penalty(line)) {
        return to_vector(n.lock()->position);
      }
    }
    // start_posを指定し続けると進まないので、とりあえず目標に進ませる
    const Eigen::Vector2d ret =
        start_pos + std::min(max_branch_length, (goal_pos - start_pos).norm()) *
                        (goal_pos - start_pos).normalized();
    return ret;
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
  // 避けきれない障害物を調べる
  if (!obstacles_.empty()) {
    // パフォーマンス向上のため，調べる範囲を限定する
    const box_t area{to_point(start - margin * Eigen::Vector2d::Ones()),
                     to_point(start + margin * Eigen::Vector2d::Ones())};

    // 実際に障害物に当たっているかを調べる
    // 近い障害物ほど優先度高めに
    for (auto itr = obstacles_.qbegin(
             boost::geometry::index::intersects(area) &&
             boost::geometry::index::nearest(to_point(start), obstacles_.size()));
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
  if (in_penalty(to_point(start)) || !boost::geometry::within(to_point(start), game_area_)) {
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
    const auto nearest_node = tree.qbegin(boost::geometry::index::nearest(to_point(sample), 1));

    // 同じ点だったら却下
    if (!boost::geometry::equals(to_point(sample), (*nearest_node)->position)) {
      // 最も近い点から一定距離を置いて点を打ち、コースに障害物がないことを確認
      const auto new_p = to_new_p(to_vector((*nearest_node)->position), sample);

      if (!is_obstructed(to_point(new_p), margin) && !in_penalty(to_point(new_p)) &&
          boost::geometry::within(to_point(new_p), area)) {
        return std::make_shared<node>(
            new_p,
            (*nearest_node)->cost + (new_p - to_vector((*nearest_node)->position)).norm(),
            *nearest_node);
      }
    }
  }
}

rrt_star::point_t rrt_star::to_point(const Eigen::Vector2d& p) const {
  return {p.x(), p.y()};
}

rrt_star::line_t rrt_star::to_line(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const {
  return {to_point(a), to_point(b)};
}

Eigen::Vector2d rrt_star::to_vector(const point_t& p) const {
  return {p.x(), p.y()};
}

rrt_star::node::node(const Eigen::Vector2d& pos, double c, const std::shared_ptr<node>& ptr)
    : position({pos.x(), pos.y()}), cost(c), parent(ptr) {}

rrt_star::object::object(const Eigen::Vector2d& pos, double radius)
    : position(pos), r(radius) {}

} // namespace planner
} // namespace ai_server
