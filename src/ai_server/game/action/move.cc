#include <cmath>
#include <boost/math/constants/constants.hpp>
#include "move.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

using boost::math::constants::pi;

namespace ai_server {
namespace game {
namespace action {

move::move(context& ctx, unsigned int id)
    : base(ctx, id),
      target_(0.0, 0.0, 0.0),
      margin_{10.0, pi<double>() / 180.0},
      finished_(false) {}

void move::move_to(double x, double y, double theta) {
  target_ = {x, y, theta};
}

void move::move_to(const Eigen::Vector2d& pos, double theta) {
  target_.head<2>() = pos;
  target_.z()       = theta;
}

void move::move_to(const Eigen::Vector3d& target) {
  target_ = target;
}

void move::set_margin(const move::margin_t& margin) {
  margin_ = margin;
}

bool move::finished() const {
  return finished_;
}

move::margin_t move::margin() const {
  return margin_;
}

Eigen::Vector3d move::target() const {
  return target_;
}

model::command move::execute() {
  const auto our_robot_team = model::our_robots(world(), team_color());
  const auto robot_p        = util::math::position3d(our_robot_team.at(id_));
  model::command command{};

  //ロボットが指定位置に存在するか
  finished_ = (robot_p - target_).head<2>().norm() <= margin_.position &&
              util::math::inferior_angle(target_.z(), robot_p.z()) <= margin_.theta;

  if (!finished_) {
    command.set_position({target_.x(), target_.y(), target_.z()});
  }

  return command;
}
} // namespace action
} // namespace game
} // namespace ai_server
