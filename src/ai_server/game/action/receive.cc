#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <Eigen/Geometry>

#include "ai_server/game/action/receive.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

using boost::math::constants::pi;

namespace ai_server {
namespace game {
namespace action {
receive::receive(context& ctx, unsigned int id)
    : base(ctx, id),
      dribble_(0),
      passer_id_(0),
      flag_(false),
      shoot_flag_(false),
      kick_type_setted_(false),
      kick_type_({model::command::kick_type_t::none, 0}),
      shoot_pos_(Eigen::Vector2d::Zero()),
      avoid_penalty_(false) {}

void receive::set_dribble(int dribble) {
  dribble_ = dribble;
}
int receive::dribble() const {
  return dribble_;
}
void receive::set_passer(unsigned int passer_id) {
  passer_id_ = passer_id;
}
unsigned int receive::passer() const {
  return passer_id_;
}
void receive::set_shoot(const Eigen::Vector2d& shoot_pos) {
  shoot_pos_  = shoot_pos;
  shoot_flag_ = true;
}
void receive::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_        = kick_type;
  kick_type_setted_ = true;
}

void receive::set_avoid_penalty(bool avoid_penalty) {
  avoid_penalty_ = avoid_penalty;
}

model::command receive::execute() {
  const auto wf = world().field();
  // ロボット半径
  constexpr double robot_rad = 90.0;
  // 既定のキックパワー
  constexpr int kick_power = 50;
  //それぞれ自機を生成
  model::command command{};
  const auto our_robots = model::our_robots(world(), team_color());
  if (!our_robots.count(id_)) return command;
  const auto& robot               = our_robots.at(id_);
  const Eigen::Vector2d robot_pos = util::math::position(robot);
  const Eigen::Vector2d ball_vel  = util::math::velocity(world().ball());
  const Eigen::Vector2d ball_pos  = util::math::position(world().ball());
  const double theta =
      shoot_flag_ ? std::atan2(shoot_pos_.y() - robot_pos.y(), shoot_pos_.x() - robot_pos.x())
                  : std::atan2(ball_vel.y(), ball_vel.x()) + pi<double>();

  // シュート時にボールの直線から離れる距離
  const double kick_dist = shoot_flag_ ? 100.0 : 0.0;
  const Eigen::Rotation2Dd rot{theta};
  const Eigen::Vector2d kick_pos = robot_pos + kick_dist * (rot * Eigen::Vector2d::UnitX());

  //ボールがめっちゃ近くに来たら受け取ったと判定
  //現状だとボールセンサに反応があるか分からないので
  if ((robot_pos - ball_pos).norm() < 100.0) flag_ = true;

  if (ball_vel.norm() >= 500.0) {
    for (double dist = (kick_pos - ball_pos).norm(); dist > robot_rad; dist -= 50.0) {
      // 基準位置
      const Eigen::Vector2d base_pos = ball_pos + dist * ball_vel.normalized();
      // 目標位置
      const Eigen::Vector2d pos = base_pos - kick_dist * (rot * Eigen::Vector2d::UnitX());
      // フィールド内なら
      if (std::abs(pos.x()) < wf.x_max() + robot_rad &&
          std::abs(pos.y()) < wf.y_max() + robot_rad &&
          (!avoid_penalty_ ||
           std::abs(pos.x()) < wf.x_max() - wf.penalty_length() - robot_rad - 10.0 ||
           std::abs(pos.y()) > 0.5 * wf.penalty_width() + robot_rad + 10.0)) {
        command.set_position(pos, theta);
        break;
      }
    }
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
