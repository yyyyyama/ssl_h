#include <Eigen/Core>
#include <cmath>
#include "ai_server/game/action/chase_ball.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace action {

void chase_ball::set_target(double x, double y) {
  target_x_ = x;
  target_y_ = y;
}

model::command chase_ball::execute() {
  model::command command(id_);
  const auto fri_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& robot     = fri_robots.at(id_);

  using namespace Eigen;
  using boost::math::constants::pi;

  model::command::position_t first_pos;  // ボールからrの円周上
  model::command::position_t second_pos; // 最終位置(回り込み)
  model::command::position_t my_pos;     // 今の位置

  model::command::velocity_t next_vel; // 次の速度

  const double ball_to_r = 300.; // ボールに対する半径

  const double acc       = 1000.;                             // 加速度
  const int acc_count    = 60;                                // 加速時間
  const double acc_dist  = acc * pow(acc_count / 60., 2) / 2; // 加速にかかる距離
  const double max_vel   = acc * acc_count / 60.;             // 最高速度
  const double rot_omega = 90 * pi<double>() / 180.; // 円周上を回るときの角速度
  const double rot_vel   = ball_to_r * rot_omega;    // 円周上を回るときの速度
  const int dribble_pow  = 5;                        // ドリブルの強さ
  double move_omega      = 0.;                       // move_to_ball_回転速度 (定義のみ)

  // ボールデータ
  const double ball_x = world_.ball().x();
  const double ball_y = world_.ball().y();

  const double ball_vx = world_.ball().vx();
  const double ball_vy = world_.ball().vy();

  // 自分の位置
  my_pos = {robot.x(), robot.y(), robot.theta()};

  // 自分->ボール
  Vector2d v1(ball_x - my_pos.x, ball_y - my_pos.y);
  v1              = v1.normalized();
  double v1_theta = std::atan2(v1(1), v1(0));

  // ボール->ターゲット
  Vector2d v2(target_x_ - ball_x, target_y_ - ball_y);
  v2              = v2.normalized();
  double v2_theta = std::atan2(v2(1), v2(0));

  if (!init_flag_) {
    count_     = 0;
    sub_count_ = 0;
    init_flag_ = true;

    if ((std::abs(util::wrap_to_pi(pi<double>() + v1_theta - v2_theta))) <
        (90 * pi<double>() / 180.)) {
      wrap_flag_ = true;
      if (util::wrap_to_pi(pi<double>() + v1_theta - v2_theta) > 0) {
        sign_flag_ = true;
      }
    }
  }

  // 位置の決定
  if (!wrap_flag_) {
    first_pos = {ball_x - ball_to_r * std::cos(v1(0)), ball_y - ball_to_r * sin(v1(1)),
                 v1_theta};
  } else {
    if (sign_flag_) {
      first_pos = {ball_x - ball_to_r * std::sin(v2_theta), ball_y + ball_to_r * cos(v2_theta),
                   v2_theta - 90 * pi<double>() / 180.};
    } else {
      first_pos = {ball_x + ball_to_r * std::sin(v2_theta), ball_y - ball_to_r * cos(v2_theta),
                   v2_theta + 90 * pi<double>() / 180.};
    }
  }

  double move_theta =
      std::atan2(first_pos.y - my_pos.y, first_pos.x - my_pos.x); // move_to_ball_角度

  second_pos = {ball_x - ball_to_r * std::cos(v2(0)), ball_y - ball_to_r * std::sin(v2(1)),
                v2_theta};

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
      move_omega = move_angle / 1.;

      if (count_ <= acc_count && dist1 > first_dist_ / 2) {
        next_vel = {acc * count_ / 60. * std::cos(move_theta),
                    acc * count_ / 60. * std::sin(move_theta), move_omega};
      }

      else if (count_ > acc_count && dist1 >= acc_dist) {
        next_vel = {max_vel * std::cos(move_theta), max_vel * std::sin(move_theta), move_omega};
      }

      else if (dist1 < acc_dist && sub_count_ <= acc_count) {
        sub_count_++;
        next_vel = {acc * (acc_count - sub_count_) / 60. * std::cos(move_theta),
                    acc * (acc_count - sub_count_) / 60. * std::sin(move_theta), move_omega};
      }

      // 位置調整
      else {
        next_vel = {(first_pos.x - my_pos.x) / 5., (first_pos.y - my_pos.y) / 5.,
                    move_angle / 5.};

        if (dist1 < 100) {
          mode_      = mode::wraparound;
          count_     = 0;
          sub_count_ = 0;
        }
      }
      break;

    // 回り込み
    case mode::wraparound:
      count_++;
      if (count_ <= acc_count) {
        if (wrap_angle >= 0) {
          next_vel = {rot_vel * std::sin(move_theta), -rot_vel * std::cos(move_theta),
                      rot_omega};
        } else {
          next_vel = {-rot_vel * std::sin(move_theta), rot_vel * std::cos(move_theta),
                      -rot_omega};
        }
      }

      //位置調整
      else if (dist2 > 50 || std::abs(wrap_angle) > 2 * pi<double>() / 180.) {
        next_vel = {(second_pos.x - my_pos.x) / 5., (second_pos.y - my_pos.y) / 5.,
                    wrap_angle / 5.};
      } else {
        next_vel = {0, 0, 0};
        mode_    = mode::dribble;
      }
      break;

    // ドリブルしながら直進
    case mode::dribble:
      next_vel = {300 * std::cos(v2_theta), 300 * std::sin(v2_theta), 0};
      command.set_dribble(dribble_pow);
      fin_flag_ = true;
      break;
  }

  // ボールの速度を加算
  next_vel = {next_vel.vx + ball_vx, next_vel.vy + ball_vy, next_vel.omega};
  command.set_velocity(next_vel);
  return command;
}

bool chase_ball::finished() const {
  return fin_flag_;
}

} // namespace ai_server
} // namespace game
} // namespace action
