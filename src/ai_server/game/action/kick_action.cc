#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "ai_server/util/math.h"
#include "ai_server/util/math/to_vector.h"
#include "kick_action.h"

#include <iostream>

namespace ai_server {
namespace game {
namespace action {

void kick_action::kick_to(double x, double y) {
  x_ = x;
  y_ = y;
}

void kick_action::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_ = kick_type;
}

// 蹴れる位置に移動するときに蹴る目標位置を見ているかボールを見ているか指定する関数
void kick_action::set_mode(mode mod) {
  mode_ = mod;
}

void kick_action::set_dribble(int dribble) {
  dribble_ = dribble;
}

void kick_action::set_angle_margin(double margin) {
  margin_ = margin;
}

kick_action::state kick_action::get_state() {
  return state_;
}

model::command kick_action::execute() {
  using boost::math::constants::pi;
  using boost::math::constants::two_pi;

  finishflag_ = false;

  const auto our_robots    = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& robot_me     = our_robots.at(id_);
  const double robot_x     = robot_me.x();
  const double robot_y     = robot_me.y();
  const double robot_theta = util::wrap_to_2pi(robot_me.theta());

  const auto ball     = world_.ball();
  const double ball_x = ball.x();
  const double ball_y = ball.y();

  // ボールから目標位置のx成分
  const double to_target_x = x_ - ball_x;
  // ボールから目標位置のy成分
  const double to_target_y = y_ - ball_y;
  // ボールから目標位置のx成分
  const double to_robot_x = robot_x - ball_x;
  // ボールからロボットのy成分
  const double to_robot_y = robot_y - ball_y;
  // ボールから目標位置の角度 0から2π
  const double atand1 = util::wrap_to_2pi(std::atan2(to_target_y, to_target_x));
  // ボールからロボットの角度 0から2π
  const double atand2 = util::wrap_to_2pi(std::atan2(to_robot_y, to_robot_x));
  const double atand3 = util::wrap_to_2pi(atand2 + pi<double>());
  const double dth    = std::abs(atand1 - atand2) - pi<double>();

  model::command command(id_);
  model::command::position_t robot_pos;

  // ボールがこの速度を超えたら終了
  const double finish_velocity = 2500;

  const double direction1 = atand3;
  const double direction2 = mode_ == mode::goal ? atand1 : atand3;
  const double dist       = dribble_ != 3 ? 200 : 250;

  std::cerr << util::math::velocity(ball).norm() << std::endl;
  if (!std::isnan(ball.vx()) && !std::isnan(ball.vy()) &&
      util::math::velocity(ball).norm() > finish_velocity && advanceflag_) {
    // ボールが一定速度以上になったら蹴ったと判定
    robot_pos = {robot_x, robot_y, robot_theta};
    command.set_position(robot_pos);
    finishflag_  = true;
    state_       = state::finished;
    advanceflag_ = false;
  } else if (std::hypot(to_robot_x, to_robot_y) > dist && !aroundflag_) {
    // ロボットがボールから250以上離れていればボールに近づく処理
    state_    = state::move;
    robot_pos = {ball_x, ball_y, direction1};
    command.set_position(robot_pos);
    if (dribble_ != 0) command.set_dribble(dribble_);

  } else if (std::abs(util::wrap_to_pi(atand2 + pi<double>() - robot_theta)) > 0.25) {
    // 位置をそのままにロボットがボールを蹴れる向きにする処理
    robot_pos = {robot_x, robot_y, util::wrap_to_2pi(atand2 + pi<double>())};
    command.set_position(robot_pos);
    if (dribble_ != 0) command.set_dribble(dribble_);
  } else if (std::abs(dth) > margin_ && state_ != state::kick) {
    // ロボット、ボール、蹴りたい位置が一直線に並んでいなければボールを中心にまわる処理
    aroundflag_ = std::hypot(to_robot_x, to_robot_y) < 350;
    if (util::wrap_to_pi(atand1 - atand2) > 0) {
      // 時計回り
      const double param = std::abs(dth) > 0.20 ? 400 : 50;
      const double si    = std::sin(atand2) * param;
      const double co    = -std::cos(atand2) * param;
      double adjustment  = util::wrap_to_pi(atand2 + pi<double>() - robot_theta);
      std::cout << "adjustment" << adjustment << std::endl;
      adjustment = adjustment > 0.02 ? adjustment * 8 : adjustment * 3;
      if (std::isnan(robot_me.vx()) || std::isnan(robot_me.vy())) {
        command.set_velocity({si, co, -param / std::hypot(to_robot_x, to_robot_y) / 1.3});
      } else {
        command.set_velocity(
            {si, co,
             -std::hypot(robot_me.vx(), robot_me.vy()) / std::hypot(to_robot_x, to_robot_y) +
                 adjustment});
      }
    } else {
      // 反時計回り
      const double param = std::abs(dth) > 0.20 ? 400 : 50;
      const double si    = -std::sin(atand2) * param;
      const double co    = std::cos(atand2) * param;
      double adjustment  = util::wrap_to_pi(atand2 + pi<double>() - robot_theta);
      std::cout << "adjustment" << adjustment << std::endl;
      adjustment = adjustment > 0.02 ? adjustment * 8 : adjustment * 3;
      if (std::isnan(robot_me.vx()) || std::isnan(robot_me.vy())) {
        command.set_velocity({si, co, param / std::hypot(to_robot_x, to_robot_y) / 1.3});
      } else {
        command.set_velocity(
            {si, co,
             std::hypot(robot_me.vx(), robot_me.vy()) / std::hypot(to_robot_x, to_robot_y) +
                 adjustment});
      }
    }
    if (dribble_ != 0) command.set_dribble(dribble_);
  } else {
    // キックフラグをセットし、ボールの位置まで移動する処理
    state_ = state::kick;
    // command.set_velocity(
    //     {200 * std::cos(robot_me.theta()), 200 * std::sin(robot_me.theta()), 0});
    command.set_velocity(
        {-200 * std::cos(atand2), -200 * std::sin(atand2), 0});
    // robot_pos = {ball_x, ball_y, atand1};
    // command.set_position(robot_pos);
    command.set_kick_flag(kick_type_);
    advanceflag_ = true;
  }

  return command;
};

bool kick_action::finished() const {
  return finishflag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
