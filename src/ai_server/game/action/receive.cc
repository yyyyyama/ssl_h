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
      approaching_flag_(false) {}

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
  kick_type_ = kick_type;
}

model::command receive::execute() {
  //それぞれ自機を生成
  model::command command(id_);

  command.set_kick_flag(kick_type_);
  command.set_dribble(dribble_);

  // 最大速度
  constexpr double max_vel = 5000.0;
  // 最小速度
  constexpr double min_vel = 0.0;
  // 遅延時間
  constexpr double delay = 0.0;

  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if (!our_robots.count(id_)) return command;
  const auto& robot               = our_robots.at(id_);
  const Eigen::Vector2d robot_vel = util::math::velocity(robot);
  const Eigen::Vector2d robot_pos = util::math::position(robot) + robot_vel * delay;
  const Eigen::Vector2d ball_vel  = util::math::velocity(world_.ball());
  const Eigen::Vector2d ball_pos  = util::math::position(world_.ball()) + ball_vel * delay;
  const double theta =
      shoot_flag_ ? std::atan2(shoot_pos_.y() - robot_pos.y(), shoot_pos_.x() - robot_pos.x())
                  : std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x());
  const double omega = 4.0 * util::math::wrap_to_pi(theta - robot.theta());

  // シュート時にボールの直線から離れる距離
  const double kick_dist = (robot_pos - ball_pos).norm() > 120 ? 100.0 : 70.0;

  //ボールがめっちゃ近くに来たら受け取ったと判定
  //現状だとボールセンサに反応があるか分からないので
  if (shoot_flag_) {
    if ((robot_pos - ball_pos).norm() < 100) {
      approaching_flag_ = true;
    } else if (approaching_flag_) {
      flag_ = true;
      return command;
    }
  } else if ((robot_pos - ball_pos).norm() < 100) {
    flag_ = true;
    return command;
  }

  bool no_kicker_flag     = !our_robots.count(passer_id_);
  const auto passer_theta = no_kicker_flag ? 0 : our_robots.at(passer_id_).theta();

  Eigen::Vector2d normalize; // 正面に移動したい対象の単位ベクトル

  if (ball_vel.norm() < 500) { // passerの正面に移動したい
    normalize = Eigen::Vector2d{std::cos(passer_theta), std::sin(passer_theta)};
  } else { // ボールの移動予測地点に移動したい
    normalize = ball_vel.normalized();
  }

  // 目標位置
  const Eigen::Vector2d target_pos =
      ball_pos + normalize * (robot_pos - ball_pos).norm() -
      kick_dist * (shoot_flag_ ? Eigen::Vector2d(std::cos(theta), std::sin(theta))
                               : Eigen::Vector2d::Zero());
  // ボールから遠ざからないようにするための基準位置
  const Eigen::Vector2d base_pos =
      target_pos + kick_dist * (shoot_flag_ ? Eigen::Vector2d(std::cos(theta), std::sin(theta))
                                            : Eigen::Vector2d::Zero());
  // 目標への速度を決める係数
  const double coe =
      std::clamp(100.0 * std::pow((target_pos - robot_pos).norm(), 0.3), min_vel, max_vel);
  // 速度
  const Eigen::Vector2d vel =
      (target_pos - robot_pos).normalized() * coe + ball_vel -
      (base_pos - ball_pos).normalized() * ball_vel.dot((base_pos - ball_pos).normalized());
  command.set_velocity({vel.x(), vel.y(), omega});
  return command;
}
bool receive::finished() const {
  return flag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
