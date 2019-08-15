#include "ai_server/game/action/turn_kick.h"

#include <cmath>
#include <boost/math/constants/constants.hpp>
namespace ai_server {
namespace game {
namespace action {

turn_kick::turn_kick(const model::world& world, bool is_yellow, unsigned int id)
    : base(world, is_yellow, id),
      kick_type_({model::command::kick_type_t::none, 0.0}),
      flag_(false),
      ready_flag_(false),
      near_post_({world.field().x_max(), -world.field().goal_width() / 2}),
      shoot_range_(250.0) {}

model::command turn_kick::execute() {
  using boost::math::constants::pi;
  model::command command(id_);

  const auto robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball   = world_.ball();

  // ボールの位置
  const Eigen::Vector2d ball_pos(ball.x(), ball.y());
  // ボールの速度
  const Eigen::Vector2d ball_vel(ball.vx(), ball.vy());

  // ロボットが見えなかったら止める
  if (!robots.count(id_)) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  const auto& robot = robots.at(id_);

  // ロボットの位置
  const Eigen::Vector2d robot_pos(robot.x(), robot.y());

  // ロボットの近いゴールポストに対しての角度
  const double b_to_np_theta =
      std::atan2(near_post_.y() - ball_pos.y() + shoot_range_, near_post_.x() - ball_pos.x());

  // ボール前まで動く
  if (!ready_flag_) {
    // ボール前へ向かう
    const Eigen::Vector2d robot_setpos(
        world_.field().x_max() - world_.field().penalty_length() - 600.0, 0.0);
    command.set_position({robot_setpos.x(), robot_setpos.y(), -b_to_np_theta});

    // ロボットとボールの距離が短くなったらボール前と判定
    ready_flag_ = (robot_pos - ball_pos).norm() < 1000.0;
    return command;
  }

  // ボール直前かどうか
  const bool spin_flag = (ball_pos - robot_pos).norm() < 120;
  // ボール直前ならスピードを上げる
  const Eigen::Vector2d vel =
      (spin_flag ? 1000.0 : 500.0) * (ball_pos - robot_pos).normalized();
  // ボール直前なら逆方向に回転する
  const double omega =
      spin_flag
          ? 4.0 * util::math::wrap_to_pi(std::atan2(-500.0 - robot_pos.y(),
                                                    world_.field().x_max() - robot_pos.x()) -
                                         robot.theta())
          : 4.0 * util::math::wrap_to_pi(std::atan2(300.0 - robot_pos.y(),
                                                    world_.field().x_max() - robot_pos.x()) -
                                         robot.theta());

  command.set_velocity({vel.x(), vel.y(), omega});

  // ボールを蹴ったと判断したら終了
  flag_ = ball_vel.norm() > 4000.0 ||
          ball_pos.x() > world_.field().x_max() - world_.field().penalty_length() + 600;

  command.set_kick_flag({model::command::kick_type_t::line, 50});
  command.set_dribble(5);

  return command;
}

bool turn_kick::finished() const {
  return flag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
