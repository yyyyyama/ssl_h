#include <algorithm>
#include <cmath>
#include <boost/math/constants/constants.hpp>

#include "ai_server/game/action/move.h"
#include "ai_server/game/action/with_planner.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/human_like.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "stopgame.h"

using boost::math::constants::pi;
using namespace std::chrono_literals;

namespace ai_server {
namespace game {
namespace agent {

// 通常stopgame用コンストラクタ
stopgame::stopgame(context& ctx, const std::vector<unsigned int>& ids)
    : base(ctx), ids_(ids), nearest_robot_(0) {}

std::vector<std::shared_ptr<action::base>> stopgame::execute() {
  std::vector<std::shared_ptr<action::base>> baseaction;
  const auto our_robots = model::our_robots(world(), team_color());
  const auto ene_robots = model::enemy_robots(world(), team_color());
  std::vector<unsigned int> visible_ids;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
               [&our_robots](unsigned int i) { return our_robots.count(i); });
  if (ids_.empty() || visible_ids.empty()) return baseaction;

  const Eigen::Vector2d ball_pos = util::math::position(world().ball());
  const double ballxsign  = ((ball_pos.x() > 0) || (std::abs(ball_pos.x()) < 250)) ? 1.0 : -1.0;
  const double ballysign  = ((ball_pos.y() > 0) || (std::abs(ball_pos.y()) < 250)) ? 1.0 : -1.0;
  constexpr double margin = 700.0;

  nearest_robot_ = *std::min_element(
      visible_ids.cbegin(), visible_ids.cend(), [&ball_pos, &our_robots](auto& a, auto& b) {
        return (util::math::position(our_robots.at(a)) - ball_pos).norm() <
               (util::math::position(our_robots.at(b)) - ball_pos).norm();
      });

  const double dist = (2 * world().field().y_max() -
                       std::abs(world().field().y_max() - std::abs(ball_pos.y()))) /
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
            margin * (Eigen::Vector2d(world().field().x_min(), 0.0) - ball_pos).normalized();
      } else {
        // それ以外
        tmp_pos.x() = ball_pos.x() - i * 500;
        tmp_pos.y() = ball_pos.y() - dist * ballysign * (i % 2 == 0 ? -i : i) / 2;
        if (std::abs(tmp_pos.y()) > world().field().y_max())
          tmp_pos.y() = ball_pos.y() - dist * ballysign * (i - 1);
        if (std::abs(tmp_pos.x()) > world().field().x_max() - 300)
          tmp_pos.x() = ballxsign * (world().field().x_max() - 300);
        if ((std::abs(tmp_pos.x()) >
             world().field().x_max() - (world().field().penalty_length() + 500)) &&
            (std::abs(tmp_pos.y()) < world().field().penalty_width() / 2.0 + 500)) {
          tmp_pos.x() =
              ballxsign * (world().field().x_max() - (world().field().penalty_length() + 600));
        }
        i++;
      }
      target_pos[id] = tmp_pos;
    }
  }

  // path_planner, action設定
  // 障害物としてのロボット半径
  constexpr double obs_robot_rad = 300.0;
  // フィールドから出られる距離
  constexpr double field_margin = 200.0;
  // ペナルティエリアから余裕を持たせる距離
  constexpr double penalty_margin = 150.0;

  planner::obstacle_list common_obstacles;
  {
    for (const auto& ene : ene_robots) {
      common_obstacles.add(
          model::obstacle::point{util::math::position(ene.second), obs_robot_rad});
    }
    common_obstacles.add(model::obstacle::point{ball_pos, margin});
    common_obstacles.add(model::obstacle::enemy_penalty_area(world().field(), penalty_margin));
    common_obstacles.add(model::obstacle::our_penalty_area(world().field(), penalty_margin));
  }
  for (auto id : visible_ids) {
    const Eigen::Vector2d robot_pos = util::math::position(our_robots.at(id));
    const double theta = std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x());
    {
      auto obstacles = common_obstacles;
      for (const auto& our : our_robots) {
        if (our.first != id)
          obstacles.add(model::obstacle::point{util::math::position(our.second), 300.0});
      }
    }
    auto move = make_action<action::move>(id);
    move->move_to(target_pos[id], theta);
    // 自チームロボットを障害物設定
    planner::obstacle_list obstacles = common_obstacles;
    for (const auto& our : our_robots) {
      if (our.first == id) continue;
      obstacles.add(model::obstacle::point{util::math::position(our.second), obs_robot_rad});
    }
    // planner::human_likeを使用
    std::unique_ptr<planner::human_like> hl = std::make_unique<planner::human_like>();
    hl->set_area(world().field(), field_margin);
    baseaction.push_back(
        std::make_shared<action::with_planner>(move, std::move(hl), obstacles));
  }
  return baseaction;
}
} // namespace agent
} // namespace game
} // namespace ai_server
