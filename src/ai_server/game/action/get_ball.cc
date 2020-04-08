#include <cmath>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/math/constants/constants.hpp>

#include "ai_server/planner/rrt_star.h"
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
      target_(target),
      kick_margin_(200),
      kick_type_({model::command::kick_type_t::line, 45}),
      manual_kick_flag_(true),
      allow_(10) {}

get_ball::get_ball(const model::world& world, bool is_yellow, unsigned int id)
    : get_ball(world, is_yellow, id, Eigen::Vector2d(world.field().x_max(), 0.0)) {}

get_ball::running_state get_ball::state() const {
  return state_;
}

void get_ball::set_target(double x, double y) {
  target_ = Eigen::Vector2d{x, y};
}

void get_ball::set_kick_margin(double margin) {
  kick_margin_ = margin;
}

void get_ball::set_kick_type(const model::command::kick_flag_t& kick_type) {
  manual_kick_flag_ = true;
  kick_type_        = kick_type;
}

void get_ball::set_allow(double allow) {
  allow_ = allow;
}

Eigen::Vector2d get_ball::target() const {
  return target_;
}

void get_ball::set_pow(int pow) {
  manual_kick_flag_       = false;
  std::get<1>(kick_type_) = pow;
}

void get_ball::set_chip(bool chip) {
  manual_kick_flag_ = false;
  std::get<0>(kick_type_) =
      chip ? model::command::kick_type_t::chip : model::command::kick_type_t::line;
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

  if ((ball_pos - target_).norm() < allow_) {
    state_ = running_state::finished;
    return command;
  }

  // ロボットとボールが十分近づいたと判定する距離
  constexpr double robot_rad = 125;
  // 他ロボットからとる距離
  constexpr double obs_robot_rad = 150.0;
  // ドリブルバーの回転速度
  constexpr int dribble_value = 5;
  // ロボットとボールの距離
  const double dist_b_to_r = (robot_pos - ball_pos).norm();
  // ロボットと目標の距離
  const double dist_r_to_t = (robot_pos - target_).norm();

  // 敵とボールの奪い合いになるか
  const bool battle_flag =
      dist_b_to_r < 1000.0 &&
      std::any_of(enemy_robots.cbegin(), enemy_robots.cend(), [&ball_pos](const auto& ene) {
        return (util::math::position(ene.second) - ball_pos).norm() < 500.0;
      });

  // ボールの方を向くか
  const bool to_ball_flag = battle_flag && dist_b_to_r > robot_rad;

  const double theta =
      to_ball_flag ? std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x())
                   : std::atan2(target_.y() - robot_pos.y(), target_.x() - robot_pos.x());
  const double omega =
      std::clamp(3.0 * util::math::wrap_to_pi(theta - robot.theta()), -3.0, 3.0);

  // 速度指令
  switch (state_) {
    // ボールを蹴る
    case running_state::dribble: {
      if (!battle_flag || dist_b_to_r > 200.0) {
        state_ = running_state::move;
      }

      const Eigen::Vector2d vel = 2.0 * (ball_pos - robot_pos);
      const Eigen::Vector2d tar = (ball_pos - robot_pos).norm() * vel.normalized() + robot_pos;

      // 障害物設定
      planner::rrt_star rrt{};
      rrt.set_area(world_.field(), 200.0);
      const planner::position_t robot_p{robot_pos.x(), robot_pos.y(), robot.theta()};
      const planner::position_t target_p{tar.x(), tar.y(), theta};
      planner::obstacle_list obstacles;
      for (const auto& r : enemy_robots) {
        obstacles.add(model::obstacle::point{util::math::position(r.second), obs_robot_rad});
      }

      const auto plan        = rrt.planner();
      const auto plan_result = plan(robot_p, target_p, obstacles);

      const Eigen::Vector2d final_vel =
          vel.norm() *
          (util::math::position(std::get<0>(plan_result)) - robot_pos).normalized();
      command.set_velocity({final_vel.x(), final_vel.y(), omega});
      break;
    }

    // 移動
    default: {
      if (battle_flag && dist_b_to_r < 300.0) state_ = running_state::dribble;

      const double rad =
          std::abs(util::math::wrap_to_pi(
              robot.theta() - std::atan2(ball_pos.y() - robot_pos.y(),
                                         ball_pos.x() - robot_pos.x()))) < 0.2 * pi<double>()
              ? 90.0
              : 150.0;
      Eigen::Vector2d pos = ball_pos + rad * (ball_pos - target_).normalized();
      if (to_ball_flag) {
        pos = ball_pos + rad * (robot_pos - ball_pos).normalized();
      } else if (std::abs(util::math::wrap_to_pi(
                     std::atan2(robot_pos.y() - ball_pos.y(), robot_pos.x() - ball_pos.x()) -
                     std::atan2(pos.y() - ball_pos.y(), pos.x() - ball_pos.x()))) >
                 0.3 * pi<double>()) {
        const auto [p1, p2] = util::math::calc_isosceles_vertexes(robot_pos, ball_pos, rad);
        pos                 = ((p1 - pos).norm() < (p2 - pos).norm()) ? p1 : p2;
      }
      const Eigen::Vector2d vel =
          3.5 * (pos - robot_pos) +
          (dist_b_to_r > robot_rad ? ball_vel : Eigen::Vector2d::Zero());
      const Eigen::Vector2d tar = (pos - robot_pos).norm() * vel.normalized() + robot_pos;

      // 障害物設定
      planner::rrt_star rrt{};
      rrt.set_area(world_.field(), 200.0);
      const planner::position_t robot_p{robot_pos.x(), robot_pos.y(), robot.theta()};
      const planner::position_t target_p{tar.x(), tar.y(), theta};

      planner::obstacle_list obstacles;
      for (const auto& r : our_robots) {
        if (r.first != id_)
          obstacles.add(model::obstacle::point{util::math::position(r.second), obs_robot_rad});
      }
      for (const auto& r : enemy_robots) {
        obstacles.add(model::obstacle::point{util::math::position(r.second),
                                             battle_flag ? 80.0 : obs_robot_rad});
      }

      const auto plan        = rrt.planner();
      const auto plan_result = plan(robot_p, target_p, obstacles);

      const Eigen::Vector2d final_vel =
          vel.norm() *
          (util::math::position(std::get<0>(plan_result)) - robot_pos).normalized();
      command.set_velocity({final_vel.x(), final_vel.y(), omega});
    }
  }

  // キック・ドリブル
  {
    const Eigen::Vector2d tmp(robot_pos.x() + dist_r_to_t * std::cos(robot.theta()),
                              robot_pos.y() + dist_r_to_t * std::sin(robot.theta()));
    const boost::geometry::model::segment<Eigen::Vector2d> line =
        boost::geometry::model::segment(robot_pos, tmp);
    if (dist_b_to_r < robot_rad && boost::geometry::distance(line, target_) < kick_margin_)
      kick(robot_pos, enemy_robots, command);
    if (dist_b_to_r < robot_rad) command.set_dribble(dribble_value);
  }
  return command;
}

void get_ball::kick(const Eigen::Vector2d& robot_pos,
                    const std::unordered_map<unsigned int, model::robot>& enemy_robots,
                    model::command& command) {
  if (manual_kick_flag_) {
    command.set_kick_flag(kick_type_);
    return;
  }
  // チップが有効そうな距離
  constexpr double radius = 1000.0;
  //自分から1000の位置と自分で三角形を作り,間に敵が1つでもあったらチップにする
  const bool flag = std::any_of(
      enemy_robots.cbegin(), enemy_robots.cend(), [this, &robot_pos](const auto& it) {
        const Eigen::Vector2d ene_pos = util::math::position(it.second);
        return (ene_pos - robot_pos).norm() < radius &&
               std::abs(util::math::wrap_to_pi(std::abs(
                   std::atan2(ene_pos.y() - robot_pos.y(), ene_pos.x() - robot_pos.x()) -
                   std::atan2(target_.y() - robot_pos.y(), target_.x() - robot_pos.x())))) <
                   1.0;
      });
  if (flag) {
    command.set_kick_flag({model::command::kick_type_t::chip, std::get<1>(kick_type_)});
  } else {
    command.set_kick_flag(kick_type_);
  }
}

bool get_ball::finished() const {
  return state_ == running_state::finished;
}
} // namespace action
} // namespace game
} // namespace ai_server
