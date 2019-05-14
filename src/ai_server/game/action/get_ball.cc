#include <cmath>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/math/constants/constants.hpp>

#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/geometry.h"
#include "ai_server/util/math/geometry_traits.h"
#include "ai_server/util/math/to_vector.h"
#include "get_ball.h"

namespace bg  = boost::geometry;
using polygon = bg::model::polygon<Eigen::Vector2d>;
using boost::math::constants::pi;
using namespace std::chrono_literals;

namespace ai_server {
namespace game {
namespace action {

get_ball::get_ball(const model::world& world, bool is_yellow, unsigned int id,
                   const Eigen::Vector2d& target)
    : base(world, is_yellow, id),
      state_(running_state::move),
      round_flag_(false),
      target_(target),
      final_target_(target),
      pow_(40),
      kick_margin_(50),
      chip_(false),
      allow_(10) {}

get_ball::get_ball(const model::world& world, bool is_yellow, unsigned int id)
    : get_ball(world, is_yellow, id, Eigen::Vector2d(world.field().x_max(), 0.0)) {}

get_ball::running_state get_ball::state() const {
  return state_;
}

void get_ball::set_target(double x, double y) {
  final_target_ = Eigen::Vector2d{x, y};
  target_       = Eigen::Vector2d{x, y};
}

void get_ball::set_pow(double pow) {
  pow_ = pow;
}

void get_ball::set_kick_margin(double margin) {
  kick_margin_ = margin;
}

void get_ball::set_chip(bool chip) {
  chip_ = chip;
}

void get_ball::set_allow(double allow) {
  allow_ = allow;
}

Eigen::Vector2d get_ball::target() const {
  return target_;
}

double get_ball::pow() const {
  return pow_;
}

bool get_ball::chip() const {
  return chip_;
}

model::command get_ball::execute() {
  model::command command{id_};
  const auto our_robots   = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  if (!our_robots.count(id_)) return command;
  const auto& robot               = our_robots.at(id_);
  const Eigen::Vector2d robot_pos = util::math::position(robot);
  const Eigen::Vector2d ball_pos  = util::math::position(world_.ball());
  const Eigen::Vector2d ball_vel  = util::math::velocity(world_.ball());

  if ((ball_pos - final_target_).norm() < allow_) {
    state_ = running_state::finished;
    return command;
  }

  // 押してドリブルしてキック
  mode_ = place_mode::push;

  if ((ball_pos - final_target_).norm() - (ball_pos + ball_vel - final_target_).norm() <
          -1000 &&
      ball_pos.x() + ball_vel.x() < world_.field().x_max()) {
    // ボールに速度が乗ってるときは速度方向に回り込む
    target_ = ball_pos - ball_vel;
  } else {
    target_ = final_target_;
  }

  // ドリブルバーの回転速度
  constexpr int dribble_value = 5;
  // ボールから目標への角度
  auto theta = mode_ == place_mode::push || mode_ == place_mode::pass
                   ? std::atan2(target_.y() - ball_pos.y(), target_.x() - ball_pos.x())
                   : std::atan2(ball_pos.y() - target_.y(), ball_pos.x() - target_.x());
  // ロボットとボールの距離
  const double dist_b_to_r = (robot_pos - ball_pos).norm();
  // ボールと目標の距離
  const double dist_b_to_t = (ball_pos - target_).norm();
  // ロボットと目標の距離
  const double dist_r_to_t = (robot_pos - target_).norm();
  // ボールと目標を通る直線
  auto ball_to_target_f = [this, &ball_pos](double x) {
    return ((target_.y() - ball_pos.y()) / (target_.x() - ball_pos.x())) * (x - ball_pos.x()) +
           ball_pos.y();
  };
  // b基準のaの符号
  auto sign = [](double a, double b) { return ((a > b) - (a < b)); };

  switch (state_) {
    // ボールを蹴る
    case running_state::dribble: {
      target_ = final_target_;
      if (dist_b_to_r > 600.0 ||
          std::abs(util::math::wrap_to_pi(
              robot.theta() - std::atan2(ball_pos.y() - robot_pos.y(),
                                         ball_pos.x() - robot_pos.x()))) > pi<double>() / 6.0) {
        // 取り損ねたら状態をmoveに戻す
        state_ = running_state::move;
        return command;
      }
      const double coef         = mode_ == place_mode::pull ? 700.0 : 1000.0;
      const Eigen::Vector2d vel = coef * (ball_pos - robot_pos) / (ball_pos - robot_pos).norm();
      const double omega        = 4.0 * util::math::wrap_to_pi(theta - robot.theta());

      const double d = (target_ - robot_pos).norm();
      Eigen::Vector2d tmp(robot_pos.x() + d * std::cos(robot.theta()),
                          robot_pos.y() + d * std::sin(robot.theta()));
      if ((tmp - target_).norm() < kick_margin_) {
        kick(robot_pos, enemy_robots, command, pow_, chip_);
      } else {
        command.set_kick_flag(
            model::command::kick_flag_t{model::command::kick_type_t::none, 0});
      }
      command.set_dribble(dribble_value);
      command.set_velocity({vel.x(), vel.y(), omega});
      first_ball_ = ball_pos;
      break;
    }

    // ボールを持つ
    case running_state::hold: {
      if (dist_b_to_r > 600.0 ||
          std::abs(util::math::wrap_to_pi(
              robot.theta() - std::atan2(ball_pos.y() - robot_pos.y(),
                                         ball_pos.x() - robot_pos.x()))) > pi<double>() / 6.0) {
        // 取り損ねたら状態をmoveに戻す
        state_ = running_state::move;
        return command;
      }
      if ((robot_pos - first_ball_).norm() < 80.0 || dist_b_to_r < 90.0) {
        // ロボットがボールの初期位置に来たら配置に移行する
        state_ = running_state::dribble;
        return command;
      }
      constexpr double coef     = 1500.0;
      const Eigen::Vector2d vel = ball_vel + coef * (ball_pos - robot_pos).normalized();
      const double omega        = 4.0 * util::math::wrap_to_pi(theta - robot.theta());

      const double d = (target_ - robot_pos).norm();
      Eigen::Vector2d tmp(robot_pos.x() + d * std::cos(robot.theta()),
                          robot_pos.y() + d * std::sin(robot.theta()));
      if ((tmp - target_).norm() < kick_margin_) {
        kick(robot_pos, enemy_robots, command, pow_, chip_);
      } else {
        command.set_kick_flag(
            model::command::kick_flag_t{model::command::kick_type_t::none, 0});
      }
      command.set_dribble(dribble_value);
      command.set_velocity({vel.x(), vel.y(), omega});
    } break;

    // 移動
    default: {
      if (std::abs(util::math::wrap_to_pi(
              robot.theta() -
              std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x()))) <
              pi<double>() / 40.0 &&
          dist_b_to_r < 400.0) {
        // ドリブルバーの正面にボールが来たら状態をholdに移行
        state_      = running_state::hold;
        round_flag_ = false;
        first_ball_ = ball_pos;
        return command;
      }

      state_                     = running_state::move;
      const Eigen::Vector2d base = ball_pos;
      //基準と目標の距離
      const double dist_ba_to_r = (base - robot_pos).norm();
      if (((120.0 < dist_ba_to_r && dist_ba_to_r < 500.0) || round_flag_) &&
          dist_ba_to_r < 800.0) {
        // 回り込み
        round_flag_      = dist_r_to_t - 100.0 < dist_b_to_t;
        const int r_sign = (mode_ == place_mode::pull ? 1 : -1) *
                           sign(robot_pos.y(), ball_to_target_f(robot_pos.x())) *
                           sign(target_.x(), ball_pos.x());
        const double round_coef =
            100.0 +
            550.0 *
                std::abs(util::math::wrap_to_pi(
                    std::atan2(target_.y() - robot_pos.y(), target_.x() - robot_pos.x()) -
                    std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x())));
        // ボールを中心に回転する速度
        const double vx =
            round_coef *
            std::sin(std::atan2(robot_pos.y() - base.y(), robot_pos.x() - base.x())) * r_sign;
        const double vy =
            -round_coef *
            std::cos(std::atan2(robot_pos.y() - base.y(), robot_pos.x() - base.x())) * r_sign;
        // ボールへ向かう速度
        const double to_ball_coef = (robot_pos - ball_pos).norm() > 250.0 ? 500.0 : 100.0;
        const Eigen::Vector2d vel = to_ball_coef * (ball_pos - robot_pos).normalized();
        const double omega        = 4.0 * util::math::wrap_to_pi(theta - robot.theta());
        command.set_velocity({ball_vel.x() + vx + vel.x(), ball_vel.y() + vy + vel.y(), omega});
      } else {
        const Eigen::Vector2d pos(
            ball_pos.x() - (dist_b_to_r > dist_b_to_t ? 150.0 : 450.0) * std::cos(theta),
            ball_pos.y() - (dist_b_to_r > dist_b_to_t ? 150.0 : 450.0) * std::sin(theta));
        command.set_position({pos.x(), pos.y(), theta});
      }
    }
  }
  return command;
}

void get_ball::kick(const Eigen::Vector2d& robot_pos,
                    const std::unordered_map<unsigned int, model::robot>& enemy_robots,
                    model::command& command, const double pow, const bool chip) {
  constexpr double radius = 1000.0;
  const auto tmp = (robot_pos - target_).norm() / radius; //自位置 - 撃つ目標位置の比
  const auto ratio = 1 - tmp;
  //これで自分から1000の点が出せる
  decltype(robot_pos) pos = (-ratio * robot_pos + target_) / tmp;
  constexpr auto mergin   = 200.0;
  const auto shift_p      = util::math::calc_isosceles_vertexes(pos, robot_pos, mergin);
  polygon poly;
  bg::exterior_ring(poly) = {pos, std::get<0>(shift_p), std::get<1>(shift_p), pos};
  //自分から1000の位置と自分で三角形を作り,間に敵が1つでもあったらチップにする
  bool flag = std::any_of(enemy_robots.cbegin(), enemy_robots.cend(), [&poly](const auto& it) {
    const Eigen::Vector2d p((it.second).x(), (it.second).y());
    return !bg::disjoint(p, poly);
  });
  if (flag || chip) {
    command.set_kick_flag(model::command::kick_flag_t{model::command::kick_type_t::chip, pow});
  } else {
    command.set_kick_flag(model::command::kick_flag_t{model::command::kick_type_t::line, pow});
  }
}

bool get_ball::finished() const {
  return state_ == running_state::finished;
}
} // namespace action
} // namespace game
} // namespace ai_server
