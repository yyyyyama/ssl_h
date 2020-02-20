#include <cmath>
#include <boost/math/constants/constants.hpp>

#include "goal_keep.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/geometry.h"
#include "ai_server/util/math/to_vector.h"

using boost::math::constants::pi;

namespace ai_server::game::action {

goal_keep::goal_keep(const model::world& world, bool is_yellow, unsigned int id)
    : base(world, is_yellow, id),
      dribble_(0),
      kick_type_(model::command::kick_type_t::none, 0.0),
      halt_flag_(false) {}

void goal_keep::set_kick_type(const model::command::kick_flag_t& kick_type) {
  kick_type_ = kick_type;
}
void goal_keep::set_dribble(int dribble) {
  dribble_ = dribble;
}
void goal_keep::set_halt(bool halt_flag) {
  halt_flag_ = halt_flag;
}
int goal_keep::dribble() const {
  return dribble_;
}
model::command goal_keep::execute() {
  //それぞれ自機を生成
  model::command command(id_);

  command.set_kick_flag(kick_type_);
  command.set_dribble(dribble_);

  // ロボットの半径
  constexpr double robot_rad = 90.0;
  // ロボットの中心からキッカーまでの距離
  constexpr double to_kicker_dist = 75.0;
  // ゴールの幅
  const double width = world_.field().goal_width();
  // 遅延時間
  constexpr double delay = 0.08;

  const auto wf = world_.field();

  const auto our_robots   = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();

  if (!our_robots.count(id_) || halt_flag_) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  const auto robot = our_robots.at(id_);
  const Eigen::Vector2d goal_pos(world_.field().x_min(), 0);
  const Eigen::Vector2d robot_vel = util::math::velocity(robot);
  const Eigen::Vector2d robot_pos = util::math::position(robot) + robot_vel * delay;
  const Eigen::Vector2d ball_vel  = util::math::velocity(world_.ball());
  const Eigen::Vector2d ball_pos  = util::math::position(world_.ball()) + ball_vel * delay;
  const double omega              = 4.0 * util::math::wrap_to_pi(0 - robot.theta());

  const auto& target_robots = enemy_robots;
  unsigned int kicker       = 0;
  bool no_kicker_flag       = enemy_robots.empty();
  const auto kicker_itr =
      std::min_element(enemy_robots.cbegin(), enemy_robots.cend(), [&](auto& a, auto& b) {
        return (util::math::position(a.second) - ball_pos).norm() <
               (util::math::position(b.second) - ball_pos).norm();
      });

  if (kicker_itr == enemy_robots.cend()) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  kicker                = kicker_itr->first;
  const auto kicker_pos = no_kicker_flag ? Eigen::Vector2d(0, 0)
                                         : Eigen::Vector2d(target_robots.at(kicker).x(),
                                                           target_robots.at(kicker).y());
  const auto kicker_theta = no_kicker_flag ? 0 : target_robots.at(kicker).theta();
  // ボールの直線
  auto f = [&goal_pos, &ball_vel, &ball_pos, &kicker_pos, &kicker_theta,
            &no_kicker_flag](double x) {
    if ((goal_pos - ball_pos).normalized().dot(ball_vel) > 1000) {
      return (ball_vel.y() / ball_vel.x()) * (x - ball_pos.x()) + ball_pos.y();
    } else if (!no_kicker_flag && ((kicker_pos - ball_pos).norm() < 1000) &&
               (kicker_pos - goal_pos).norm() > (ball_pos - goal_pos).norm() + 50 &&
               std::abs(
                   std::atan2(ball_pos.y() - kicker_pos.y(), ball_pos.x() - kicker_pos.x()) -
                   kicker_theta) < pi<double>() / 3.0) {
      return std::tan(kicker_theta) * (x - ball_pos.x()) + ball_pos.y();
    } else {
      return ((goal_pos.y() - ball_pos.y()) / (goal_pos.x() - ball_pos.x())) *
                 (x - ball_pos.x()) +
             ball_pos.y();
    }
  };

  const double x = wf.x_min() + robot_rad;
  const double y = std::clamp(f(x + to_kicker_dist), -width / 2.0, width / 2.0);
  const Eigen::Vector2d pos(x, y);
  const Eigen::Vector2d vel =
      4.0 * (pos - robot_pos) +
      Eigen::Vector2d(0.0, (std::abs(y) < 0.8 * width / 2.0 ? ball_vel.y() : 0.0));
  command.set_velocity({vel.x(), vel.y(), omega});
  return command;
}
bool goal_keep::finished() const {
  return false;
}
} // namespace ai_server::game::action
