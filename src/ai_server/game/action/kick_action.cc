#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "ai_server/util/math.h"
#include "kick_action.h"

namespace ai_server {
namespace game {
namespace action {

kick_action::kick_action(const model::world& world, bool is_yellow, unsigned int id)
    : base(world, is_yellow, id) {
  const auto ball = world.ball();
  old_ball_x      = ball.x();
  old_ball_y      = ball.y();
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

void kick_action::set_anglemargin(double margin) {
  margin_ = margin;
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

  // 角度を調整するときの許容誤差(rad)
  // const double margin = 0.01;
  // executeが呼ばれる間にボールがこれだけ移動したら蹴ったと判定する長さ(mm)
  const double kick_decision = 60;

  const double direction1 = mode_ == mode::goal ? robot_theta : atand3;
  const double direction2 = mode_ == mode::goal ? atand1 : atand3;
  double dis              = 300;
  if (dribble_ != 3) {
    dis = 250;
  }

  if (std::hypot(old_ball_x - ball_x, old_ball_y - ball_y) > kick_decision && advanceflag_) {
    // executeが呼ばれる間の時間でボールが一定以上移動していたら蹴ったと判定
    robot_pos    = {robot_x, robot_y, robot_theta};
    finishflag_  = true;
    advanceflag_ = false;
  } else if (std::hypot(to_robot_x, to_robot_y) > dis && !aroundflag_) {
    // ロボットがボールから250以上離れていればボールに近づく処理
    robot_pos = {ball_x, ball_y, direction1};
    if (dribble_ != 33) command.set_dribble(dribble_);
  } else if (std::abs(dth) > margin_) {
    // ロボット、ボール、蹴りたい位置が一直線に並んでいなければボールを中心にまわる処理
    aroundflag_ = std::hypot(to_robot_x, to_robot_y) < 350;
    if (util::wrap_to_pi(atand1 - atand2) > 0) {
      // 時計回り
      robot_pos = {robot_x + std::abs(dth) * 200 * std::sin(atand2 - 0.20),
                   robot_y - std::abs(dth) * 200 * std::cos(atand2 - 0.20), direction2};
    } else {
      // 反時計回り
      robot_pos = {robot_x - std::abs(dth) * 200 * std::sin(atand2 + 0.20),
                   robot_y + std::abs(dth) * 200 * std::cos(atand2 + 0.20), direction2};
    }
    if (dribble_ != 33) command.set_dribble(dribble_);
  } else if (std::abs(atand1 - robot_theta) > margin_ &&
             std::abs(atand1 - robot_theta) < two_pi<double>() - margin_) {
    // 位置をそのままにロボットがボールを蹴れる向きにする処理
    aroundflag_ = false;
    robot_pos   = {robot_x, robot_y, atand1};
    if (dribble_ != 33) command.set_dribble(dribble_);
  } else {
    // キックフラグをセットし、ボールの位置まで移動する処理
    robot_pos = {ball_x, ball_y, atand1};
    command.set_kick_flag(kick_type_);
    advanceflag_ = true;
  }

  // 前のボールの位置を更新
  old_ball_x = ball_x;
  old_ball_y = ball_y;

  command.set_position(robot_pos);
  return command;
};

bool kick_action::finished() const {
  return finishflag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
