#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include "ai_server/game/action/move.h"
#include "ai_server/game/action/with_planner.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/human_like.h"
#include "ai_server/util/math/to_vector.h"

#include "kick_off_waiter.h"

namespace ai_server::game::agent {

kick_off_waiter::kick_off_waiter(context& ctx, const std::vector<unsigned int>& ids)
    : base(ctx), ids_(ids), mode_(kickoff_mode::attack) {}

std::vector<std::shared_ptr<action::base>> kick_off_waiter::execute() {
  std::vector<std::shared_ptr<action::base>> exe;
  const auto our_robots = model::our_robots(world(), team_color());
  const auto ene_robots = model::enemy_robots(world(), team_color());
  const auto wf         = world().field();
  const auto ball_pos   = util::math::position(world().ball());

  // 視認可能なロボットをids_から抽出する
  std::vector<unsigned int> visible_ids;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
               [&our_robots](auto id) { return our_robots.count(id); });
  if (visible_ids.empty()) return exe;

  // ロボットの障害物としての半径
  const double obs_robot_rad = 200.0;
  // フィールド外に出られる距離
  const double area_margin = 200.0;
  // 障害物設定
  planner::obstacle_list common_obstacles;
  common_obstacles.add(model::obstacle::center_circle(wf, 150.0));
  common_obstacles.add(model::obstacle::enemy_penalty_area(wf, 150.0));
  common_obstacles.add(model::obstacle::our_penalty_area(wf, 150.0));
  common_obstacles.add(model::obstacle::point{ball_pos, 650.0});
  for (const auto& robot : ene_robots) {
    common_obstacles.add(
        model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
  }

  if (mode_ == kickoff_mode::attack) {
    // 攻撃側
    // ロボットの間隔
    constexpr double interval = 500.0;
    // xは-200固定
    constexpr double x_line = -200.0;

    // 何番目のロボットか判別
    int count_a = 2;
    int count_b = 2;
    for (auto id : visible_ids) {
      const auto robot_pos = util::math::position(our_robots.at(id));
      const Eigen::Vector2d a_pos(x_line, wf.y_min() + interval * count_a);
      const Eigen::Vector2d b_pos(x_line, wf.y_min() - interval * count_b);

      auto move = make_action<action::move>(id);
      if ((a_pos - robot_pos).norm() < (b_pos - robot_pos).norm()) {
        move->move_to(a_pos, 0);
        count_a++;
      } else {
        move->move_to(b_pos, 0);
        count_b++;
      }

      auto hl = std::make_unique<planner::human_like>();
      hl->set_area(wf, area_margin);
      auto obstacles = common_obstacles;
      for (const auto& robot : our_robots) {
        if (robot.first != id)
          obstacles.add(
              model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
      }
      exe.push_back(std::make_shared<action::with_planner>(move, std::move(hl), obstacles));
    }
  } else {
    // 守備側
    auto tmp_ids = visible_ids;
    std::vector<unsigned int> ene_robots_id;
    for (auto& enemy : ene_robots) ene_robots_id.push_back(enemy.first);

    // 敵キッカーをマーク候補から除外
    const auto ball_near_it = std::min_element(
        ene_robots_id.cbegin(), ene_robots_id.cend(), [&ene_robots, &ball_pos](auto a, auto b) {
          return (util::math::position(ene_robots.at(a)) - ball_pos).norm() <
                 (util::math::position(ene_robots.at(b)) - ball_pos).norm();
        });
    // 敵キッカーの座標を取得し,マーキング候補から除外
    double enemy_theta = 0;
    double enemy_x     = 0;
    double enemy_y     = 0;
    if (ball_near_it != ene_robots_id.end()) {
      const auto& enemy = ene_robots.at(*ball_near_it);
      enemy_theta       = enemy.theta();
      enemy_x           = enemy.x();
      enemy_y           = enemy.y();
      ene_robots_id.erase(ball_near_it);
    }

    // 常に敵キッカーの目の前にいるような位置取りにする
    // センターサークルに入らないような半径
    constexpr double r = 750;
    const double x     = std::min(-std::abs(r * std::cos(enemy_theta)), -100.0);
    const double y     = r * std::sin(enemy_theta);
    const double theta = std::atan2(enemy_y - y, enemy_x - x);
    const auto it =
        std::min_element(tmp_ids.cbegin(), tmp_ids.cend(), [&our_robots, x, y](auto a, auto b) {
          return std::hypot(our_robots.at(a).x() - x, our_robots.at(a).y() - y) <
                 std::hypot(our_robots.at(b).x() - x, our_robots.at(b).y() - y);
        });
    if (it != tmp_ids.end()) {
      auto move = make_action<action::move>(*it);
      move->move_to(x, y, theta);

      auto hl = std::make_unique<planner::human_like>();
      hl->set_area(wf, area_margin);
      auto obstacles = common_obstacles;
      for (const auto& robot : our_robots) {
        if (robot.first != *it)
          obstacles.add(
              model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
      }
      exe.push_back(std::make_shared<action::with_planner>(move, std::move(hl), obstacles));
      tmp_ids.erase(it);
    }

    // 敵をセンターラインに近い順にソート
    std::sort(ene_robots_id.begin(), ene_robots_id.end(), [&ene_robots](auto a, auto b) {
      return ene_robots.at(a).x() < ene_robots.at(b).x();
    });

    // 敵をセンターラインに近い順にマーク
    for (std::size_t i = 0; i < visible_ids.size() - 1; ++i) {
      // マーク位置の決定
      if (tmp_ids.empty() || (ene_robots_id.begin() + i) == ene_robots_id.end()) return exe;
      const double y         = ene_robots.at(ene_robots_id[i]).y();
      constexpr double x     = -700;
      constexpr double theta = 0;

      // マークするロボットの決定
      const auto it = std::min_element(
          tmp_ids.cbegin(), tmp_ids.cend(), [&our_robots, x, y](auto a, auto b) {
            return std::hypot(our_robots.at(a).x() - x, our_robots.at(a).y() - y) <
                   std::hypot(our_robots.at(b).x() - x, our_robots.at(b).y() - y);
          });
      auto move = make_action<action::move>(*it);
      move->move_to(x, y, theta);

      auto hl = std::make_unique<planner::human_like>();
      hl->set_area(wf, area_margin);
      auto obstacles = common_obstacles;
      for (const auto& robot : our_robots) {
        if (robot.first != *it)
          obstacles.add(
              model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
      }
      exe.push_back(std::make_shared<action::with_planner>(move, std::move(hl), obstacles));
      tmp_ids.erase(it);
    }
  }

  return exe;
}

void kick_off_waiter::set_mode(const kickoff_mode& mode) {
  mode_ = mode;
}

kick_off_waiter::kickoff_mode kick_off_waiter::mode() const {
  return mode_;
}

} // namespace ai_server::game::agent
