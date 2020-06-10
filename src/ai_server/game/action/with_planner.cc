#include <Eigen/Core>

#include "ai_server/planner/base.h"
#include "ai_server/util/math/to_vector.h"
#include "with_planner.h"

namespace ai_server::game::action {

bool with_planner::finished() const {
  return action_->finished();
}

model::command with_planner::execute() {
  auto cmd = action_->execute();

  if (auto sp  = cmd.setpoint_pair();
      auto pos = std::get_if<model::setpoint::position>(&std::get<0>(sp))) {
    const auto robot = our_robots(world(), team_color()).at(id());
    const auto start = util::math::position(robot);
    const auto goal  = Eigen::Vector2d(std::get<0>(*pos), std::get<1>(*pos));

    const auto planner = planner_->planner();
    const auto new_pos = planner(start, goal, obstacles_);
    cmd.set_position(std::get<0>(new_pos));
  }

  return cmd;
}

} // namespace ai_server::game::action
