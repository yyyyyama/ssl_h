#include <cmath>

#include <Eigen/Geometry>

#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "chase_ball.h"

namespace ai_server::game::action {

chase_ball::chase_ball(context& ctx, unsigned int id, const Eigen::Vector2d& target)
    : base(ctx, id),
      mode_(mode::move_to_ball),
      kick_target_(target),
      start_dist_(0),
      ball_pos_(util::math::position(world().ball())) {}

chase_ball::chase_ball(context& ctx, unsigned int id)
    : chase_ball(ctx, id, Eigen::Vector2d(ctx.world.field().x_max(), 0.0)) {}

void chase_ball::set_target(double x, double y) {
  kick_target_ = {x, y};
}

void chase_ball::set_target(const Eigen::Vector2d& target) {
  kick_target_ = target;
}

enum chase_ball::mode chase_ball::mode() const {
  return mode_;
}

model::command chase_ball::execute() {
  model::command command{};
  const auto fri_robots = model::our_robots(world(), team_color());
  const auto& robot     = fri_robots.at(id_);

  using boost::math::constants::half_pi;
  using boost::math::constants::pi;

  // ボールに対する半径
  constexpr double ball_to_r = 200;
  // 加速度
  constexpr double acc = 2000;
  // 加速時間
  constexpr int acc_count = 60;
  // 加速にかかる距離
  const double acc_dist = acc * std::pow(acc_count / 60.0, 2) / 2;
  // 最高速度
  constexpr double max_vel = acc * acc_count / 60;
  // ドリブルの強さ
  constexpr int dribble_pow = 5;

  // ボールデータ
  const auto ball_pos = util::math::position(world().ball());
  auto ball_vel       = util::math::velocity(world().ball());
  if (ball_vel.norm() > 4000.0) ball_vel = 4000.0 * ball_vel.normalized();

  // 自分の位置
  const auto my_pos = util::math::position(robot);

  //自分とボールの距離
  const double dist = (my_pos - ball_pos).norm();

  // 自分とボールの角度
  const double v1_theta = std::atan2(ball_pos.y() - my_pos.y(), ball_pos.x() - my_pos.x());

  // ボールとターゲットの角度
  const double v2_theta =
      std::atan2(kick_target_.y() - ball_pos.y(), kick_target_.x() - ball_pos.x());

  if (!init_flag_) {
    count_     = 0;
    sub_count_ = 0;
    init_flag_ = true;

    if (std::abs(util::math::wrap_to_pi(v1_theta - v2_theta)) > half_pi<double>()) {
      wrap_flag_ = true;
      if (util::math::wrap_to_pi(v1_theta - v2_theta) < 0) sign_flag_ = true;
    }
  }

  if (ball_vel.norm() > 400) wrap_flag_ = true;

  // mode::wait_ballの選択
  if (mode_ == mode::wait_ball) {
    if (ball_vel.norm() < 300 ||
        std::abs(util::math::wrap_to_pi(pi<double>() + v1_theta -
                                        std::atan2(ball_vel.y(), ball_vel.x()))) >
            90 * pi<double>() / 180) {
      // 角度の変更が出来るようにhalf_piを使っていない
      mode_      = mode::move_to_ball;
      init_flag_ = false;
      wait_flag_ = false;
    }
  } else {
    if (ball_vel.norm() >= 1000) {
      if (std::abs(util::math::wrap_to_pi(pi<double>() + v1_theta -
                                          std::atan2(ball_vel.y(), ball_vel.x()))) <
          30 * pi<double>() / 180) {
        mode_       = mode::wait_ball;
        start_dist_ = dist;
        wait_flag_  = true;
        ball_pos_   = ball_pos;
      }
    }
  }

  // ボールからrの円周上
  Eigen::Vector2d first_pos;
  double first_theta;
  // 位置の決定
  if (wait_flag_) {
    first_pos   = ball_pos + dist * ball_vel.normalized();
    first_theta = pi<double>() + std::atan2(ball_vel.y(), ball_vel.x());
  } else if (wrap_flag_) {
    if (sign_flag_) {
      first_pos   = ball_pos + ball_to_r * (Eigen::Rotation2Dd(v1_theta - half_pi<double>()) *
                                          Eigen::Vector2d::UnitX());
      first_theta = v2_theta;
    } else {
      first_pos   = ball_pos + ball_to_r * (Eigen::Rotation2Dd(v1_theta + half_pi<double>()) *
                                          Eigen::Vector2d::UnitX());
      first_theta = v2_theta;
    }
  } else {
    first_pos =
        ball_pos - ball_to_r * (Eigen::Rotation2Dd(v2_theta) * Eigen::Vector2d::UnitX());
    first_theta = v2_theta;
  }

  const double second_theta = ball_vel.norm() < 400.0
                                  ? v2_theta
                                  : std::atan2(ball_vel.y(), ball_vel.x()) + pi<double>();
  // 最終位置(回り込み)
  const Eigen::Vector2d second_pos =
      ball_pos - ball_to_r * (Eigen::Rotation2Dd(second_theta) * Eigen::Vector2d::UnitX());

  // それぞれの距離
  const double dist1 = (my_pos - first_pos).norm();
  const double dist2 = (my_pos - second_pos).norm();

  // 回転角度
  const double move_angle = util::math::wrap_to_pi(first_theta - robot.theta());
  const double wrap_angle = util::math::wrap_to_pi(second_theta - robot.theta());

  // move_to_ball 方向
  const Eigen::Vector2d move_dir = (first_pos - my_pos).normalized();

  // 次の速度
  Eigen::Vector2d next_vel;
  double omega;

  switch (mode_) {
    // first_posに移動
    case mode::move_to_ball:
      count_++;
      omega = move_angle;

      if (count_ <= acc_count && dist1 > start_dist_ / 2) {
        next_vel = acc * count_ / 60 * move_dir;
      } else if (count_ > acc_count && dist1 >= acc_dist - 100) {
        next_vel = max_vel * move_dir;
      } else if (dist1 < acc_dist && sub_count_ <= acc_count / 1.5) {
        sub_count_++;
        next_vel = acc * (acc_count - sub_count_) / 60 * move_dir;
      } else {
        if (wrap_flag_) {
          next_vel = max_vel * move_dir;
          if (dist1 < acc_dist / 2) {
            mode_      = mode::wraparound;
            count_     = 0;
            sub_count_ = 0;
          }
        } else {
          if (dist1 < acc_dist && sub_count_ <= acc_count) {
            sub_count_++;
            next_vel = acc * (acc_count - sub_count_) / 60 * move_dir;
          } else if (dist2 > 50) {
            //位置調整
            next_vel = first_pos - my_pos;
          } else {
            next_vel = Eigen::Vector2d::Zero();
            mode_    = mode::dribble;
          }
        }
      }
      break;

    // ボールに回り込む
    case mode::wraparound:
      count_++;
      omega = wrap_angle;
      //位置調整
      if (dist2 > 50) {
        next_vel = second_pos - my_pos;
      } else {
        next_vel = Eigen::Vector2d::Zero();
        mode_    = mode::dribble;
        count_   = 0;
      }
      break;

    // ドリブルしながら直進
    case mode::dribble:
      omega = 0.0;
      if (dist < 500) {
        next_vel   = 300.0 * (Eigen::Rotation2Dd(v1_theta) * Eigen::Vector2d::UnitX());
        wait_flag_ = true;
      } else {
        mode_ = mode::wraparound;
      }
      command.set_dribble(dribble_pow);
      fin_flag_ = true;
      break;

    // ボールが来るのを待つ
    case mode::wait_ball:
      count_++;
      omega = move_angle;
      if (count_ <= acc_count && dist1 > start_dist_ / 2) {
        next_vel = acc * count_ / 60 * move_dir;
      } else if (count_ > acc_count && dist1 >= 500) {
        next_vel = max_vel * move_dir;
      } else if (dist1 < acc_dist && sub_count_ <= acc_count) {
        sub_count_++;
        next_vel = acc * (acc_count - sub_count_) / 60 * move_dir;
      } else {
        // 位置調整
        next_vel = first_pos - my_pos;
      }
      break;
  }

  // ボールの速度を加算
  if (!wait_flag_) next_vel += ball_vel;
  command.set_velocity(next_vel, omega);
  return command;
}

bool chase_ball::finished() const {
  return fin_flag_;
}

} // namespace ai_server::game::action
