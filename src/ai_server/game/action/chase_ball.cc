#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include "ai_server/game/action/chase_ball.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace action {

void chase_ball::set_target(double x, double y) {
  kick_target_x_ = x;
  kick_target_y_ = y;
}

model::command chase_ball::execute() {
  model::command command(id_);
  const auto fri_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& robot     = fri_robots.at(id_);

  using namespace Eigen;
  using boost::math::constants::pi;
  using boost::math::constants::half_pi;

  model::command::position_t first_pos;  // ボールからrの円周上
  model::command::position_t second_pos; // 最終位置(回り込み)

  model::command::velocity_t next_vel; // 次の速度

  const double ball_to_r = 200; // ボールに対する半径

  const double acc       = 2000;                                   // 加速度
  const int acc_count    = 60;                                     // 加速時間
  const double acc_dist  = acc * std::pow(acc_count / 60., 2) / 2; // 加速にかかる距離
  const double max_vel   = acc * acc_count / 60;                   // 最高速度
  const double rot_omega = 180 * pi<double>() / 180.; // 円周上を回るときの角速度
  const double rot_vel   = ball_to_r * rot_omega;     // 円周上を回るときの速度
  const int dribble_pow  = 5;                         // ドリブルの強さ
  double move_omega      = 0.; // move_to_ball_回転速度 (定義のみ)
  double wrap_omega      = 0.;
  const double a         = 1;

  // ボールデータ
  const double ball_x = world_.ball().x();
  const double ball_y = world_.ball().y();

  double ball_vx = world_.ball().vx();
  double ball_vy = world_.ball().vy();

  if (std::hypot(ball_vx, ball_vy) > 4000) {
    ball_vx = ball_vx_;
    ball_vy = ball_vy_;
  } else {
    ball_vx_ = ball_vx;
    ball_vx_ = ball_vx;
  }

  // 自分の位置
  model::command::position_t my_pos = {robot.x(), robot.y(), robot.theta()};

  double dist = std::hypot(my_pos.x - ball_x, my_pos.y - ball_y);

  // 自分->ボール
  const auto v1   = Vector2d{ball_x - my_pos.x, ball_y - my_pos.y}.normalized();
  double v1_theta = std::atan2(v1(1), v1(0));

  // ボール->ターゲット
  const auto v2   = Vector2d{kick_target_x_ - ball_x, kick_target_y_ - ball_y}.normalized();
  double v2_theta = std::atan2(v2(1), v2(0));

  if (!init_flag_) {
    count_     = 0;
    sub_count_ = 0;
    init_flag_ = true;

    if (std::abs(util::wrap_to_pi(v1_theta - v2_theta)) > half_pi<double>()) {
      wrap_flag_ = true;
      if (util::wrap_to_pi(v1_theta - v2_theta) < 0) {
        sign_flag_ = true;
      }
    }
  }

  if (std::hypot(ball_vx, ball_vy) > 400) {
    wrap_flag_ = true;
  }
  // mode::wait_ballの選択
  if (mode_ == mode::wait_ball) {
    if (std::hypot(ball_vx, ball_vy) < 300 ||
        std::abs(util::wrap_to_pi(pi<double>() + v1_theta - std::atan2(ball_vy, ball_vx))) >
            90 * pi<double>() / 180) {
      mode_      = mode::move_to_ball;
      init_flag_ = false;
      wait_flag_ = false;
    }
  } else {
    if (std::hypot(ball_vx, ball_vy) >= 1000) {
      if (std::abs(util::wrap_to_pi(pi<double>() + v1_theta - std::atan2(ball_vy, ball_vx))) <
          30 * pi<double>() / 180) {
        mode_       = mode::wait_ball;
        start_dist_ = dist;
        wait_flag_  = true;
        ball_x_     = ball_x;
        ball_y_     = ball_y;
      }
    }
  }

  // 位置の決定
  if (wait_flag_) {
    first_pos = {ball_x_ + dist * std::cos(std::atan2(ball_vy, ball_vx)),
                 ball_y_ + dist * std::sin(std::atan2(ball_vy, ball_vx)),
                 pi<double>() + std::atan2(ball_vy, ball_vx)};
  } else if (wrap_flag_) {
    if (sign_flag_) {
      first_pos = {ball_x + ball_to_r * std::sin(v1_theta),
                   ball_y - ball_to_r * std::cos(v1_theta), v2_theta};
    } else {
      first_pos = {ball_x - ball_to_r * std::sin(v1_theta),
                   ball_y + ball_to_r * std::cos(v1_theta), v2_theta};
    }
  } else {
    first_pos = {ball_x - ball_to_r * std::cos(v2_theta),
                 ball_y - ball_to_r * std::sin(v2_theta), v2_theta};
  }

  double move_theta =
      std::atan2(first_pos.y - my_pos.y, first_pos.x - my_pos.x); // move_to_ball_角度

  if (std::hypot(ball_vx_, ball_vy_) < 400) {
    second_pos = {ball_x - ball_to_r * std::cos(v2_theta),
                  ball_y - ball_to_r * std::sin(v2_theta), v2_theta};
  } else {
    second_pos = {ball_x + ball_to_r * std::cos(std::atan2(ball_vy, ball_vx)),
                  ball_y + ball_to_r * std::sin(std::atan2(ball_vy, ball_vx)),
                  std::atan2(ball_vy, ball_vx) + pi<double>()};
  }

  // それぞれの距離
  double dist1 = std::hypot(my_pos.x - first_pos.x, my_pos.y - first_pos.y);
  double dist2 = std::hypot(my_pos.x - second_pos.x, my_pos.y - second_pos.y);

  // 回転角度
  double move_angle = util::wrap_to_pi(first_pos.theta - my_pos.theta);
  double wrap_angle = util::wrap_to_pi(second_pos.theta - my_pos.theta);

  switch (mode_) {
    // first_posに移動
    case mode::move_to_ball:
      count_++;
      move_omega = move_angle;

      if (count_ <= acc_count && dist1 > start_dist_ / 2) {
        next_vel = {acc * count_ / 60 * std::cos(move_theta),
                    acc * count_ / 60 * std::sin(move_theta), move_omega};
      }

      else if (count_ > acc_count && dist1 >= acc_dist - 150) {    //default 100
        next_vel = {max_vel * std::cos(move_theta), max_vel * std::sin(move_theta), move_omega};
      } else if (dist1 < acc_dist && sub_count_ <= acc_count / 1.5) {
        sub_count_++;
        next_vel = {acc * (acc_count - sub_count_) / 60. * std::cos(move_theta),
                    acc * (acc_count - sub_count_) / 60. * std::sin(move_theta), move_omega};
      } else {
        if (wrap_flag_) {
          if (dist1 >= acc_dist / 2) {
            next_vel = {max_vel * std::cos(move_theta), max_vel * std::sin(move_theta),
                        move_omega};
          } else {
            mode_      = mode::wraparound;
            count_     = 0;
            sub_count_ = 0;
            next_vel   = {max_vel * std::cos(move_theta), max_vel * std::sin(move_theta),
                        move_omega};
          }
        } else {
          if (dist1 < acc_dist && sub_count_ <= acc_count) {
            sub_count_++;
            next_vel = {acc * (acc_count - sub_count_) / 60. * std::cos(move_theta),
                        acc * (acc_count - sub_count_) / 60. * std::sin(move_theta),
                        move_omega};
          }
          //位置調整
          else if (dist2 > 50 || std::abs(v2_theta - v1_theta) > 2 * pi<double>() / 180) {
            next_vel = {(first_pos.x - my_pos.x) / a, (first_pos.y - my_pos.y) / a, move_omega};
          } else {
            next_vel = {0, 0, 0};
            mode_    = mode::dribble;
          }
        }
      }
      break;

    // 回り込み
    case mode::wraparound:
      count_++;
      wrap_omega = wrap_angle;
      if (count_ <= acc_count) {
        next_vel = {(second_pos.x - my_pos.x) / (acc_count - count_),
                    (second_pos.y - my_pos.y) / (acc_count - count_), wrap_angle};
        // next_vel = {rot_vel * std::cos(v2_theta - rot_omega / (acc_count -  count_)) /
        // count_, rot_vel * std::sin(v2_theta - rot_omega / (acc_count -  count_)) / count_,
        // wrap_omega};
      }
      //位置調整
      if (dist2 > 50 || std::abs(v2_theta - v1_theta) > 2 * pi<double>() / 180){
        next_vel = {(second_pos.x - my_pos.x) / a, (second_pos.y - my_pos.y) / a, wrap_angle};
      } else {
        next_vel = {0, 0, 0};
        mode_    = mode::dribble;
        count_   = 0;
      }
      break;

    // ドリブルしながら直進
    case mode::dribble:
      if (dist < 500) {
        next_vel   = {300 * std::cos(v1_theta), 300 * std::sin(v1_theta), 0};
        wait_flag_ = true;
      }
      /*    else {
            next_vel = {300 * std::cos(std::atan2(kick_target_y_ - my_pos.y, kick_target_x_ - my_pos.x)),
                        300 * std::sin(std::atan2(kick_target_y_ - my_pos.y, kick_target_x_ - my_pos.x)),
                        wrap_angle};

          }
      */
      if (/*v2_theta - v1_theta > half_pi<double>() ||*/ dist > 500) {
        mode_ = mode::move_to_ball/*wrapparound*/;
      }
      command.set_dribble(dribble_pow);
      fin_flag_ = true;
      break;

    case mode::wait_ball:
      count_++;
      move_omega = move_angle;
      if (count_ <= acc_count && dist1 > start_dist_ / 2) {
        next_vel = {acc * count_ / 60. * std::cos(move_theta),
                    acc * count_ / 60. * std::sin(move_theta), move_omega};
      }

      else if (count_ > acc_count && dist1 >= 500) {
        next_vel = {max_vel * std::cos(move_theta), max_vel * std::sin(move_theta), move_omega};
      }

      else if (dist1 < acc_dist && sub_count_ <= acc_count) {
        sub_count_++;
        next_vel = {acc * (acc_count - sub_count_) / 60 * std::cos(move_theta),
                    acc * (acc_count - sub_count_) / 60 * std::sin(move_theta), move_omega};
      }
      // 位置調整
      else {
        next_vel = {(first_pos.x - my_pos.x) / a, (first_pos.y - my_pos.y) / a, move_omega};
      }
      break;
  }
  std::cout << ball_x << "    " << ball_y << std::endl;
  std::cout << first_pos.x << "    " << first_pos.y << std::endl;

  // ボールの速度を加算
  if (!wait_flag_) {
    next_vel = {next_vel.vx + ball_vx, next_vel.vy + ball_vy, next_vel.omega};
  }
  command.set_velocity(next_vel);
  return command;
}

bool chase_ball::finished() const {
  return fin_flag_;
}

} // namespace ai_server
} // namespace game
} // namespace action
