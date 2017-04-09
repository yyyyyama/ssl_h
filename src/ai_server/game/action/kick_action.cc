#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "ai_server/util/math.h"
#include "kick_action.h"

namespace ai_server {
namespace game {
namespace action {

void kick_action::kick_to(double x, double y) {
  x_         = x;
  y_         = y;
  old_ball_x = x;
  old_ball_y = y;
}

void kick_action::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_ = kick_type;
}

model::command kick_action::execute() {
  using boost::math::constants::pi;
  using boost::math::constants::two_pi;

  exeflag_ = false;

  const auto our_robots    = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& robot_me     = our_robots.at(id_);
  const double robot_x     = robot_me.x();
  const double robot_y     = robot_me.y();
  const double robot_theta = util::wrap_to_2pi(robot_me.theta());

  const auto ball     = world_.ball();
  const double ball_x = ball.x();
  const double ball_y = ball.y();

  // ボールから目標位置のx成分
  const double dx1 = x_ - ball_x;
  // ボールから目標位置のy成分
  const double dy1 = y_ - ball_y;
  // ボールから目標位置のx成分
  const double dx2 = robot_x - ball_x;
  // ボールからロボットのy成分
  const double dy2 = robot_y - ball_y;
  // ボールから目標位置の角度 0から2π
  const double atand1 = util::wrap_to_2pi(std::atan2(dy1, dx1));
  // ボールからロボットの角度 0から2π
  const double atand2 = util::wrap_to_2pi(std::atan2(dy2, dx2));
  const double dth    = std::abs(atand1 - atand2) - pi<double>();

  model::command command(id_);
  model::command::position_t robot_pos;

  // 角度を調整するときの許容誤差(rad)
  const double margin = 0.02;
  // executeが呼ばれる間にボールがこれだけ移動したら蹴ったと判定する長さ(mm)
  const double kick_decision = 5;

  if (std::hypot(old_ball_x - ball_x, old_ball_y - ball_y) > kick_decision) {
    // executeが呼ばれる間の時間でボールが一定以上移動していたら蹴ったと判定
    robot_pos = {robot_x, robot_y, robot_theta};
    exeflag_  = true;
  } else if (std::hypot(dx2, dy2) > 200) {
    // ロボットがボールから200mm以上離れていればボールに近づく処理
    robot_pos = {ball_x, ball_y, robot_theta};
  } else if (dth < -margin || dth > +margin) {
    // ロボット、ボール、蹴りたい位置が一直線に並んでいなければボールを中心にまわる処理
    if ((atand1 > atand2 && (atand1 - atand2 <= pi<double>())) ||
        (atand1 < atand2 && (atand2 - atand1 >= pi<double>()))) {
      // 時計回り
      robot_pos = {robot_x + std::abs(dth) * 300 * std::sin(atand2),
                   robot_y - std::abs(dth) * 300 * std::cos(atand2), robot_theta};
    } else {
      // 反時計回り
      robot_pos = {robot_x - std::abs(dth) * 300 * std::sin(atand2),
                   robot_y + std::abs(dth) * 300 * std::cos(atand2), robot_theta};
    }
  } else if (std::abs(atand1 - robot_theta) > margin &&
             std::abs(atand1 - robot_theta) < two_pi<double>() - margin) {
    // 位置をそのままにロボットがボールを蹴れる向きにする処理
    robot_pos = {robot_x, robot_y, atand1};
  } else {
    // キックフラグをセットし、ボールの位置まで移動する処理
    robot_pos = {ball_x, ball_y, atand1};
    command.set_kick_flag(kick_type_);
  }

  // 前のボールの位置を更新
  old_ball_x = ball_x;
  old_ball_y = ball_y;

  command.set_position(robot_pos);
  return command;
};

bool kick_action::finished() const {
  return exeflag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
