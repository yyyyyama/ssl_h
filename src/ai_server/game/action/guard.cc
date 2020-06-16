#include <algorithm>
#include <cmath>

#include "ai_server/game/action/guard.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include <boost/math/constants/constants.hpp>

using boost::math::constants::pi;

namespace ai_server {
namespace game {
namespace action {

guard::guard(context& ctx, unsigned int id)
    : base(ctx, id),
      target_{0.0, 0.0},
      shift_flag_(false),
      magnification_(5.0),
      margin_(0.0),
      decelation_(0.0),
      dribble_(0),
      kick_type_{model::command::kick_type_t::none, 0.0},
      halt_flag_(false) {}

void guard::move_to(double x, double y) {
  target_ = {x, y};
}
void guard::move_on(bool shift_flag) {
  shift_flag_ = shift_flag;
}
model::command::kick_flag_t guard::kick_type() const {
  return kick_type_;
}
void guard::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_ = kick_type;
}
void guard::set_dribble(int dribble) {
  dribble_ = dribble;
}
void guard::set_halt(bool halt_flag) {
  halt_flag_ = halt_flag;
}
void guard::set_magnification(double magnification) {
  magnification_ = magnification;
}
int guard::dribble() const {
  return dribble_;
}
model::command guard::execute() {
  //それぞれ自機を生成
  model::command command{};

  //キックフラグを立てて置く
  command.set_kick_flag(kick_type_);

  command.set_dribble(dribble_);

  const auto robots = model::our_robots(world(), team_color());
  if (!robots.count(id_) || halt_flag_) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  //座標の取得
  const Eigen::Vector2d goal(world().field().x_min(), 0.0);
  const auto& robot              = robots.at(id_);
  const Eigen::Vector2d wall_pos = util::math::position(robot);

  //ゴールからロボットの位置への角度
  const auto wall_theta = std::atan2(robot.y() - goal.y(), robot.x() - goal.x());
  //ゴールから対象の位置への角度
  const auto target_theta = std::atan2(target_.y() - goal.y(), target_.x() - goal.x());
  const auto corner_theta =
      std::atan2(world().field().penalty_width() / 2, world().field().penalty_length());

  const double robot_size = std::copysign(95, wall_pos.y());
  //ゴールから壁の距離
  const double distance = (wall_pos - goal).norm();
  //目標地点の間隔
  const double interval = 0.3;
  //速度の倍率
  const double magnification = 15000.0;
  //速度の上限
  const double limit = 1500.0;
  const double range = std::hypot(robot.x() - goal.x(), robot.y() - goal.y());

  margin_     = shift_flag_ ? 250 : 0.0;
  decelation_ = std::clamp(std::abs(corner_theta - std::abs(wall_theta)) * 10, 0.2, 1.0);

  //壁からボール方向に目標地点をずらして座標を計算
  if (range >= 2500) {
    command.set_position({goal.x() + world().field().penalty_length(), 0.0, 0.0});
    return command;
  }
  if (std::abs(wall_theta) <= corner_theta) {
    const double x = goal.x() + world().field().penalty_length() + 160.0 + margin_;
    const double y =
        std::sin(wall_theta + std::copysign(interval, target_theta - wall_theta)) * distance;
    const double theta = target_theta;
    const Eigen::Vector2d pos(x, y);
    const Eigen::Vector2d vel =
        std::min(std::abs(magnification * util::math::wrap_to_pi(target_theta - wall_theta) /
                          pi<double>()),
                 limit) *
        (pos - wall_pos).normalized() * decelation_;
    const double omega = 2.0 * util::math::wrap_to_pi(theta - robot.theta());
    command.set_velocity({vel.x(), vel.y(), omega});
  } else {
    const double x = std::max(
        goal.x() + std::cos(wall_theta + std::copysign(interval, target_theta - wall_theta)) *
                       distance,
        world().field().x_min() + 100);
    const double y = std::copysign(world().field().penalty_width() / 2, wall_pos.y()) +
                     robot_size + std::copysign(margin_, target_.y());
    const double theta = target_theta;
    const Eigen::Vector2d pos(x, y);
    const Eigen::Vector2d vel =
        std::min(std::abs(magnification * util::math::wrap_to_pi(target_theta - wall_theta) /
                          pi<double>()),
                 limit) *
        (pos - wall_pos).normalized() * decelation_;
    const double omega = 4.0 * util::math::wrap_to_pi(theta - robot.theta());
    command.set_velocity({vel.x(), vel.y(), omega});
  }
  return command;
}
bool guard::finished() const {
  return false;
}
} // namespace action
} // namespace game
} // namespace ai_server
