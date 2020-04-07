#include <algorithm>
#include <cmath>
#include <boost/math/constants/constants.hpp>

#include "ai_server/game/action/move.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/rrt_star.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "stopgame.h"

using boost::math::constants::pi;
using namespace std::chrono_literals;

namespace ai_server {
namespace game {
namespace agent {

// 通常stopgame用コンストラクタ
stopgame::stopgame(const model::world& world, bool is_yellow,
                   const std::vector<unsigned int>& ids)
    : base(world, is_yellow), ids_(ids), nearest_robot_(0) {}

std::vector<std::shared_ptr<action::base>> stopgame::execute() {
  std::vector<std::shared_ptr<action::base>> baseaction;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ene_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  std::vector<unsigned int> visible_ids;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
               [&our_robots](unsigned int i) { return our_robots.count(i); });
  if (ids_.empty() || visible_ids.empty()) return baseaction;

  const Eigen::Vector2d ball_pos = util::math::position(world_.ball());
  const double ballxsign  = ((ball_pos.x() > 0) || (std::abs(ball_pos.x()) < 250)) ? 1.0 : -1.0;
  const double ballysign  = ((ball_pos.y() > 0) || (std::abs(ball_pos.y()) < 250)) ? 1.0 : -1.0;
  constexpr double margin = 700.0;

  nearest_robot_ = *std::min_element(
      visible_ids.cbegin(), visible_ids.cend(), [&ball_pos, &our_robots](auto& a, auto& b) {
        return (util::math::position(our_robots.at(a)) - ball_pos).norm() <
               (util::math::position(our_robots.at(b)) - ball_pos).norm();
      });

  const double dist =
      (2 * world_.field().y_max() - std::abs(world_.field().y_max() - std::abs(ball_pos.y()))) /
      ids_.size();

  // 目標座標算出
  std::unordered_map<unsigned int, Eigen::Vector2d> target_pos;
  {
    int i = 2;
    for (auto id : ids_) {
      Eigen::Vector2d tmp_pos;
      if (id == nearest_robot_) {
        // 一番近いロボット
        tmp_pos =
            ball_pos +
            margin * (Eigen::Vector2d(world_.field().x_min(), 0.0) - ball_pos).normalized();
      } else {
        // それ以外
        tmp_pos.x() = ball_pos.x() - i * 500;
        tmp_pos.y() = ball_pos.y() - dist * ballysign * (i % 2 == 0 ? -i : i) / 2;
        if (std::abs(tmp_pos.y()) > world_.field().y_max())
          tmp_pos.y() = ball_pos.y() - dist * ballysign * (i - 1);
        if (std::abs(tmp_pos.x()) > world_.field().x_max() - 300)
          tmp_pos.x() = ballxsign * (world_.field().x_max() - 300);
        if ((std::abs(tmp_pos.x()) >
             world_.field().x_max() - (world_.field().penalty_length() + 500)) &&
            (std::abs(tmp_pos.y()) < world_.field().penalty_width() / 2.0 + 500)) {
          tmp_pos.x() =
              ballxsign * (world_.field().x_max() - (world_.field().penalty_length() + 600));
        }
        i++;
      }
      target_pos[id] = tmp_pos;
    }
  }

  // path_planner, action設定
  planner::obstacle_list common_obstacles;
  {
    for (const auto& ene : ene_robots) {
      common_obstacles.add(model::obstacle::point{util::math::position(ene.second), 300.0});
    }
    common_obstacles.add(model::obstacle::point{ball_pos, margin});
    common_obstacles.add(model::obstacle::enemy_penalty_area(world_.field(), 150.0));
    common_obstacles.add(model::obstacle::our_penalty_area(world_.field(), 150.0));
  }
  for (auto id : visible_ids) {
    const Eigen::Vector2d robot_pos = util::math::position(our_robots.at(id));
    const double theta = std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x());
    std::unique_ptr<planner::rrt_star> rrt = std::make_unique<planner::rrt_star>(world_);
    {
      auto obstacles = common_obstacles;
      for (const auto& our : our_robots) {
        if (our.first != id)
          obstacles.add(model::obstacle::point{util::math::position(our.second), 300.0});
      }
      rrt->set_margin(0.0);
      rrt->set_node_count(10);
      rrt->set_max_branch_length(50.0);
    }
    auto move = std::make_shared<action::move>(world_, is_yellow_, id);
    move->set_path_planner(std::move(rrt));
    // TODO: actionに障害物リストを渡す
    move->move_to(target_pos[id], theta);
    baseaction.push_back(move);
  }
  return baseaction;
}
} // namespace agent
} // namespace game
} // namespace ai_server
