#include <cmath>
#include <boost/math/constants/constants.hpp>

#include "ai_server/game/action/receive.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

using boost::math::constants::pi;

namespace ai_server {
namespace game {
namespace action {
receive::receive(const model::world& world, bool is_yellow, unsigned int id)
    : base(world, is_yellow, id),
      dribble_(0),
      passer_id_(0),
      flag_(false),
      shoot_flag_(false),
      kick_type_setted_(false),
      kick_type_({model::command::kick_type_t::none, 0}),
      shoot_pos_(Eigen::Vector2d::Zero(1, 2)) {}

void receive::set_dribble(int dribble) {
  dribble_ = dribble;
}
int receive::dribble() {
  return dribble_;
}
void receive::set_passer(unsigned int passer_id) {
  passer_id_ = passer_id;
}
unsigned int receive::passer() {
  return passer_id_;
}
void receive::set_shoot(Eigen::Vector2d shoot_pos) {
  shoot_pos_  = shoot_pos;
  shoot_flag_ = true;
}
void receive::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_        = kick_type;
  kick_type_setted_ = true;
}

model::command receive::execute() {
  // 既定のキックパワー
  constexpr int kick_power = 50;
  //それぞれ自機を生成
  model::command command(id_);
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if (!our_robots.count(id_)) return command;
  const auto& robot               = our_robots.at(id_);
  const Eigen::Vector2d robot_vel = util::math::velocity(robot);
  const Eigen::Vector2d robot_pos = util::math::position(robot);
  const Eigen::Vector2d ball_vel  = util::math::velocity(world_.ball());
  const Eigen::Vector2d ball_pos  = util::math::position(world_.ball());
  double theta =
      shoot_flag_ ? std::atan2(shoot_pos_.y() - robot_pos.y(), shoot_pos_.x() - robot_pos.x())
                  : std::atan2(ball_vel.y(), ball_vel.x()) + pi<double>();
  const double omega = 4.0 * util::math::wrap_to_pi(theta - robot.theta());

  // シュート時にボールの直線から離れる距離
  const double kick_dist = shoot_flag_ ? 100.0 : 0.0;
  const Eigen::Vector2d kick_pos =
      robot_pos + kick_dist * Eigen::Vector2d(std::cos(theta), std::sin(theta));

  //ボールがめっちゃ近くに来たら受け取ったと判定
  //現状だとボールセンサに反応があるか分からないので
  if ((robot_pos - ball_pos).norm() < 100) flag_ = true;

  if (ball_vel.norm() < 500) {
    command.set_velocity({0.0, 0.0, 0.0});
  } else {
    const Eigen::Vector2d normalize = ball_vel.normalized();

    // 基準位置
    const Eigen::Vector2d base_pos = ball_pos + normalize * (kick_pos - ball_pos).norm();
    // 目標位置
    Eigen::Vector2d target_pos =
        base_pos - kick_dist * Eigen::Vector2d(std::cos(theta), std::sin(theta));
    // 速度
    const Eigen::Vector2d vel =
        4.0 * (target_pos - robot_pos) + ball_vel -
        (base_pos - ball_pos).normalized() * ball_vel.dot((base_pos - ball_pos).normalized());
    command.set_velocity({vel.x(), vel.y(), omega});
  }
  // kick
  if (kick_type_setted_) {
    command.set_kick_flag(kick_type_);
  } else if (shoot_flag_) {
    command.set_kick_flag({model::command::kick_type_t::line, kick_power});
  } else {
    command.set_kick_flag({model::command::kick_type_t::none, 0});
  }
  // dribble
  command.set_dribble(dribble_);
  shoot_flag_       = false;
  kick_type_setted_ = false;
  return command;
}
bool receive::finished() const {
  return flag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
