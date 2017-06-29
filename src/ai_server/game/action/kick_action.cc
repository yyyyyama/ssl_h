#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "ai_server/util/math.h"
#include "ai_server/util/math/to_vector.h"
#include "kick_action.h"

#include <iostream>

namespace ai_server {
namespace game {
namespace action {

kick_action::kick_action(const model::world& world, bool is_yellow, unsigned int id)
    : base(world, is_yellow, id) {
  const auto ball = world.ball();
  old_ball_x_     = ball.x();
  old_ball_y_     = ball.y();
  mode_           = mode::goal;
  state_          = running_state::move;
  dribble_        = 0;
  margin_         = 0.2;
  finishflag_     = false;
  aroundflag_     = false;
  advanceflag_    = false;
}

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

kick_action::running_state kick_action::state() const {
  return state_;
}

model::command kick_action::execute() {
  using boost::math::constants::pi;
  using boost::math::constants::two_pi;

  const auto our_robots    = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& robot_me     = our_robots.at(id_);
  const auto robot_posi    = util::math::position(robot_me);
  const double robot_x     = robot_me.x();
  const double robot_y     = robot_me.y();
  const double robot_theta = util::wrap_to_pi(robot_me.theta());

  const auto ball     = world_.ball();
  const double ball_x = ball.x();
  const double ball_y = ball.y();
  const auto ball_vel = util::math::velocity(ball);
  const auto ball_pos = util::math::position(ball);

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
  const double dth2 = util::wrap_to_pi(std::atan2(y_-ball_y, x_-ball_x)-std::atan2(ball_y-robot_y, ball_x-robot_x));
  std::cout<<"kicker"<<id_<<"\ttarget"<<std::atan2(y_-robot_y, x_-robot_x)<<", "<<robot_theta<<", "<<util::wrap_to_pi(atand1)<<", "<<util::wrap_to_pi(atand3)<<std::endl;
  if(std::abs(dth)-std::abs(dth2)>0.0001){
	  std::cout<<"diff"<<dth<<", "<<dth2<<std::endl;
  }

  const double omega = util::wrap_to_pi(atand2-robot_theta+pi<double>());
  std::cout << "kicker" << id_ << "\t";
  std::cout << "dth " << dth << "\tatand1 " << atand1 << "\tatand2 " << atand2 << "\tatand3 "
            << atand3 <<"\ttheta"<<omega<< std::endl;
  std::cout<<"kicker\tkickto"<<x_<<", "<<y_<<std::endl;
  std::cout << "kicker" << id_ << "\t";

  model::command command(id_);
  model::command::position_t robot_pos;

  // executeが呼ばれる間にボールがこれだけ移動したら蹴ったと判定する長さ(mm)
  const double kick_decision = 40;

  const double direction1 = atand3;
  const double direction2 = mode_ == mode::goal ? atand1 : atand3;
  const double dist       = dribble_ != 3 ? 200 : 250;
  if ((std::hypot(old_ball_x_ - ball_x, old_ball_y_ - ball_y) > kick_decision &&
       advanceflag_) ||
      finishflag_) {
    // executeが呼ばれる間の時間でボールが一定以上移動していたら蹴ったと判定
    command.set_velocity({0, 0, 0});
    finishflag_  = true;
    state_       = running_state::finished;
    advanceflag_ = false;
    std::cout << "finish" << std::endl;
  } else if (std::hypot(to_robot_x, to_robot_y) > dist && !aroundflag_) {
    // ロボットがボールから250以上離れていればボールに近づく処理
    std::cout << "near" << std::endl;
    state_ = running_state::move;
    if (ball_vel.norm() < 100) {
      robot_pos = {ball_x, ball_y, direction1};
      command.set_position(robot_pos);
    } else {
      command.set_velocity({-300 * std::cos(atand2) + ball_vel.x(),
                            -300 * std::sin(atand2) + ball_vel.y(),
                            util::wrap_to_pi(atand2 + pi<double>() - robot_theta)});
    }
    if (dribble_ != 0) command.set_dribble(dribble_);

  } else if (std::abs(util::wrap_to_pi(atand2 + pi<double>() - robot_theta)) > margin_ &&
             ball_vel.norm() < 1500 && (robot_posi - ball_pos).norm() > 120) {
    // 位置をそのままにロボットがボールを蹴れる向きにする処理
    std::cout << "omega" << std::endl;
    robot_pos = {robot_x, robot_y, util::wrap_to_2pi(atand2 + pi<double>())};
    command.set_position(robot_pos);
    if (dribble_ != 0) command.set_dribble(dribble_);
  } else if (std::abs(dth) > margin_ / 2 ||
             (std::abs(dth) > margin_ / 4 && state_ != running_state::kick)) {
    // ロボット、ボール、蹴りたい位置が一直線に並んでいなければボールを中心にまわる処理
    std::cout << "round" << std::endl;
    aroundflag_ = std::hypot(to_robot_x, to_robot_y) < 350;
    if (util::wrap_to_pi(atand1 - atand2) > 0) {
      // 時計回り
      const double coe  = std::abs(dth) > 0.20 ? 400 : 50;
      const double si   = std::sin(atand2) * coe;
      const double co   = -std::cos(atand2) * coe;
      double adjustment = util::wrap_to_pi(atand2 + pi<double>() - robot_theta);
      adjustment        = adjustment > margin_ / 3 ? adjustment * 9 : adjustment * 3;
      if (std::isnan(robot_me.vx()) || std::isnan(robot_me.vy())) {
        command.set_velocity({si + ball_vel.x(), co + ball_vel.y(),
                              -coe / std::hypot(to_robot_x, to_robot_y) / 1.3});
      } else {
        command.set_velocity(
            {si + ball_vel.x(), co + ball_vel.y(),
             -std::hypot(robot_me.vx(), robot_me.vy()) / std::hypot(to_robot_x, to_robot_y) +
                 adjustment});
      }
    } else {
      // 反時計回り
      const double coe  = std::abs(dth) > 0.20 ? 400 : 50;
      const double si   = -std::sin(atand2) * coe;
      const double co   = std::cos(atand2) * coe;
      double adjustment = util::wrap_to_pi(atand2 + pi<double>() - robot_theta);
      adjustment        = adjustment > margin_ / 3 ? adjustment * 9 : adjustment * 3;
      if (std::isnan(robot_me.vx()) || std::isnan(robot_me.vy())) {
        command.set_velocity({si + ball_vel.x(), co + ball_vel.y(),
                              coe / std::hypot(to_robot_x, to_robot_y) / 1.3});
      } else {
        command.set_velocity(
            {si + ball_vel.x(), co + ball_vel.y(),
             std::hypot(robot_me.vx(), robot_me.vy()) / std::hypot(to_robot_x, to_robot_y) +
                 adjustment});
      }
    }
    if (dribble_ != 0) command.set_dribble(dribble_);
  } else {
    // キックフラグをセットし、ボールの位置まで移動する処理
    std::cout << "kick" << std::endl;
    state_           = running_state::kick;
    const double coe = std::abs(dth) > margin_ / 4 ? 100 : 0;
    const double si = std::sin(atand2) * coe * (util::wrap_to_pi(atand1 - atand2) > 0 ? 1 : -1);
    const double co = std::cos(atand2) * coe * (util::wrap_to_pi(atand1 - atand2) > 0 ? -1 : 1);
    double adjustment = std::atan2(y_ - robot_y, x_ - robot_x) - robot_theta;
    adjustment        = adjustment > 0.02 ? adjustment * 4 : adjustment * 2;
    command.set_velocity({si + -250 * std::cos(atand2) + ball_vel.x(),
                          co + -250 * std::sin(atand2) + ball_vel.y(),
                          coe / std::hypot(to_robot_x, to_robot_y) / 1.3});
    command.set_kick_flag(kick_type_);
    advanceflag_ = true;
  }

  // 前のボールの位置を更新
  old_ball_x_ = ball_x;
  old_ball_y_ = ball_y;

  return command;
};

bool kick_action::finished() const {
  return finishflag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
