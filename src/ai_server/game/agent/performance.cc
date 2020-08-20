#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <set>
#include <unordered_map>
#include <Eigen/Geometry>

#include "ai_server/game/action/vec.h"
#include "ai_server/game/action/with_planner.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/human_like.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "performance.h"

using boost::math::constants::two_pi;
using namespace std::chrono_literals;

namespace ai_server::game::agent {

performance::performance(context& ctx, const std::vector<unsigned int>& ids)
    : base(ctx),
      ids_(ids),
      start_point_(std::chrono::steady_clock::now()),
      max_rad_(1000.0),
      min_rad_(500.0),
      base_pos_{0.0, 0.0} {
  const auto now = std::chrono::steady_clock::now();
  for (const auto& id : ids_) {
    lost_point_[id] = now;
  }
}

void performance::set_area(double max_rad, double min_rad, const Eigen::Vector2d& base_pos) {
  max_rad_  = max_rad;
  min_rad_  = min_rad;
  base_pos_ = base_pos;
}

std::vector<std::shared_ptr<action::base>> performance::execute() {
  std::vector<std::shared_ptr<action::base>> baseaction;
  const auto our_robots = model::our_robots(world(), team_color());

  ///////////////////////////////////////////
  // lost判定 ///////////////////////////////
  std::vector<unsigned int> visible_ids;
  const auto now = std::chrono::steady_clock::now();
  for (const auto& a : our_robots) {
    if (lost_point_.count(a.first)) lost_point_.at(a.first) = now;
  }

  for (const auto [id, lost] : lost_point_) {
    if (now - lost <= 1s) {
      visible_ids.push_back(id);
    }
  }
  if (visible_ids.empty()) return baseaction;

  // 障害物としてのロボット半径
  constexpr double obs_robot_rad = 300.0;
  // フィールドから出られる距離
  constexpr double field_margin = 200.0;
  // ペナルティエリアから余裕を持たせる距離
  constexpr double penalty_margin = 150.0;
  // 一般障害物設定
  planner::obstacle_list common_obstacles;
  {
    for (const auto& robot : model::enemy_robots(world(), team_color())) {
      common_obstacles.add(
          model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    common_obstacles.add(model::obstacle::enemy_penalty_area(world().field(), penalty_margin));
    common_obstacles.add(model::obstacle::our_penalty_area(world().field(), penalty_margin));
  }

  ///////////////////////////////////////////
  // 目標位置を算出 /////////////////////////
  std::unordered_map<unsigned int, Eigen::Vector2d> targets;
  const double d_theta = two_pi<double>() / static_cast<double>(visible_ids.size());
  //基準点を中心に周るための角速度
  constexpr double omega0 = 1.5;
  //基準点へ収縮させる角速度
  constexpr double omega1 = 2.0;
  const double t =
      std::chrono::duration<double>{std::chrono::steady_clock::now() - start_point_}.count();
  const double rad = 0.5 * (max_rad_ - min_rad_) * (std::sin(omega1 * t) + 1.0) + min_rad_;

  for (std::size_t i = 0; i < visible_ids.size(); ++i) {
    const Eigen::Rotation2Dd rot{omega0 * t + d_theta * i};
    const Eigen::Vector2d pos = base_pos_ + rad * (rot * Eigen::Vector2d::UnitX());
    targets.emplace(visible_ids[i], pos);
  }

  for (const auto& [id, pos] : targets) {
    if (our_robots.count(id)) {
      auto action                     = make_action<action::vec>(id);
      const Eigen::Vector2d robot_pos = util::math::position(our_robots.at(id));
      const Eigen::Vector2d vel =
          (pos - robot_pos).normalized() * (1.0 * (pos - robot_pos).norm() + rad * omega0);
      action->move_at(vel, 0.0);
      // 自チームロボットを障害物設定
      planner::obstacle_list obstacles = common_obstacles;
      for (const auto& robot : our_robots) {
        if (robot.first == id) continue;
        obstacles.add(
            model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
      }
      // planner::human_likeを使用
      auto hl = std::make_unique<planner::human_like>();
      hl->set_area(world().field(), field_margin);
      baseaction.push_back(
          std::make_shared<action::with_planner>(action, std::move(hl), obstacles));
    }
  }
  return baseaction;
}
} // namespace ai_server::game::agent
