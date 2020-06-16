#include "ai_server/game/action/turn_kick.h"
#include "ai_server/util/math/to_vector.h"
#include <cmath>
#include <boost/math/constants/constants.hpp>
namespace ai_server {
namespace game {
namespace action {

turn_kick::turn_kick(context& ctx, unsigned int id)
    : base(ctx, id),
      kick_type_({model::command::kick_type_t::none, 0.0}),
      flag_(false),
      start_pos_({0, 0}),
      previous_ball_({0, 0}),
      radius_(0),
      angle_(0),
      kick_flag_(false),
      wait_flag_(false),
      state_(running_state::set) {}

model::command turn_kick::execute() {
  using boost::math::constants::pi;
  model::command command{};

  const auto robots = model::our_robots(world(), team_color());
  const auto ball   = world().ball();

  // ロボットが見えなかったら止める
  if (!robots.count(id_)) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  const auto& robot = robots.at(id_);

  // ロボットの位置
  const Eigen::Vector2d robot_pos = util::math::position(robot);

  // ボールの位置
  const Eigen::Vector2d ball_pos = util::math::position(ball);

  // ボールからロボットを見た時の初期位置の角度
  const double start_angle =
      std::atan2(ball_pos.y() - start_pos_.y(), ball_pos.x() - start_pos_.x());

  // ロボットからボールを見た時の角度
  const double theta = std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x());

  // ロボット、ボール、ロボットの目標位置が織り成す角度
  const double phi = start_angle + angle_ - theta;

  switch (state_) {
    // set - 初期位置への移動
    case running_state::set: {
      if ((start_pos_ - robot_pos).norm() < 100) {
        state_ = running_state::rotate;
        break;
      }
      command.set_position({start_pos_.x(), start_pos_.y(), theta});
      break;
    }

    // move - 指定された距離まで移動
    case running_state::move: {
      if (std::abs((robot_pos - ball_pos).norm() - radius_) < 50) {
        state_ = running_state::wait;
        break;
      }
      const Eigen::Vector2d vec = ball_pos + radius_ * (robot_pos - ball_pos).normalized();
      command.set_position({vec.x(), vec.y(), theta});
      break;
    }

    // rotate - 指定された角度まで回転
    case running_state::rotate: {
      if (std::abs(phi) < 2 * pi<double>() / 32) {
        state_ = running_state::wait;
        break;
      }
      // 目標位置までの距離
      const double length = radius_ * phi;

      // ロボットの移動方向
      const Eigen::Vector2d vec(std::sin(theta), -std::cos(theta));
      const Eigen::Vector2d no_vec     = vec.normalized();
      const Eigen::Vector2d target_pos = robot_pos + no_vec * length * 3;

      command.set_position({target_pos.x(), target_pos.y(), theta});
      break;
    }

    // wait - kick_flag_がtrueになるまで停止
    case running_state::wait: {
      // 指定位置から遠ざかったら
      if (std::abs((robot_pos - ball_pos).norm() - radius_) > 50) {
        state_     = running_state::move;
        wait_flag_ = false;
        break;
      }
      if (std::abs(phi) > 2 * pi<double>() / 32) {
        state_     = running_state::rotate;
        wait_flag_ = false;
        break;
      }

      if (kick_flag_) {
        state_     = running_state::kick;
        wait_flag_ = false;
        break;
      }

      // 指定位置に移動
      command.set_position(
          {ball_pos.x() + radius_ * std::cos(start_angle + angle_ + pi<double>()),
           ball_pos.y() + radius_ * std::sin(start_angle + angle_ + pi<double>()), theta});
      wait_flag_ = true;
      break;
    }

    // kick - キック
    case running_state::kick: {
      if ((previous_ball_ - ball_pos).norm() > 50) {
        flag_ = true;
      }
      command.set_kick_flag({model::command::kick_type_t::line, 50});
      command.set_position({ball_pos.x() + 5 * std::cos(start_angle + angle_ + pi<double>()),
                            ball_pos.y() + 5 * std::sin(start_angle + angle_ + pi<double>()),
                            theta});
      break;
    }
  }

  // ボールの位置を保存
  previous_ball_ = ball_pos;

  return command;
}

bool turn_kick::finished() const {
  return flag_;
}

void turn_kick::set_start_pos(Eigen::Vector2d start_pos) {
  start_pos_ = start_pos;
}

void turn_kick::set_radius(double radius) {
  radius_ = radius;
}

void turn_kick::set_angle(double angle) {
  angle_ = angle;
}

void turn_kick::set_kick(bool kick_flag) {
  kick_flag_ = kick_flag;
}

bool turn_kick::wait() const {
  return wait_flag_;
}

} // namespace action
} // namespace game
} // namespace ai_server
