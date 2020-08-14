#include <cmath>

#include <boost/math/constants/constants.hpp>

#include "ai_server/game/action/move.h"
#include "ai_server/game/action/with_planner.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/human_like.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "alignment.h"

using boost::math::constants::pi;

namespace ai_server::game::agent {

alignment::alignment(context& ctx, const std::vector<unsigned int>& ids)
    : base(ctx), ids_(ids) {}

std::vector<std::shared_ptr<action::base>> alignment::execute() {
  std::vector<std::shared_ptr<action::base>> exe;
  const auto our_robots = model::our_robots(world(), team_color());
  // 障害物としてのロボット半径
  constexpr double obs_robot_rad = 200.0;
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

  const double x0 =
      -world().field().x_max() / 2 + 110.0 * (static_cast<double>(ids_.size()) - 1.0);
  constexpr double y     = 0;
  constexpr double theta = pi<double>() * 1.5;
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    const auto id = *(ids_.begin() + i);
    if (!our_robots.count(id)) continue;
    // 自チームロボットを障害物設定
    auto obstacles = common_obstacles;
    for (const auto& robot : our_robots) {
      if (robot.first == id) continue;
      obstacles.add(model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    // planner::human_likeを使用
    std::unique_ptr<planner::human_like> hl = std::make_unique<planner::human_like>();
    hl->set_area(world().field(), field_margin);

    const double x = x0 - 220.0 * i;
    auto move      = make_action<action::move>(id);
    move->move_to(x, y, theta);
    exe.push_back(std::make_shared<action::with_planner>(move, std::move(hl), obstacles));
  }
  return exe;
}

} // namespace ai_server::game::agent
