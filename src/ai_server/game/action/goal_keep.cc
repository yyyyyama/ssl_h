#include <cmath>
#include <boost/math/constants/constants.hpp>

#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/geometry.h"
#include "ai_server/util/math/to_vector.h"

#include "goal_keep.h"

namespace ai_server::game::action {

goal_keep::goal_keep(context& ctx, unsigned int id)
    : base(ctx, id),
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
  using boost::math::constants::pi;
  //それぞれ自機を生成
  model::command command{};

  command.set_kick_flag(kick_type_);
  command.set_dribble(dribble_);

  // ロボットの半径
  constexpr double robot_rad = 90.0;
  // ロボットの中心からキッカーまでの距離
  constexpr double to_kicker_dist = 75.0;
  // ゴールの幅
  const double width = world().field().goal_width();

  const auto wf = world().field();

  const auto our_robots   = model::our_robots(world(), team_color());
  const auto enemy_robots = model::enemy_robots(world(), team_color());

  if (!our_robots.count(id_) || halt_flag_) {
    command.set_velocity(0.0, 0.0, 0.0);
    return command;
  }

  const Eigen::Vector2d goal_pos(world().field().x_min(), 0.0);
  const Eigen::Vector2d ball_vel = util::math::velocity(world().ball());
  const Eigen::Vector2d ball_pos = util::math::position(world().ball());

  // ボールの直線
  auto f = [&enemy_robots, &goal_pos, &ball_vel, &ball_pos](double x) {
    // ボールの速度が出ていればボール軌道
    if ((goal_pos - ball_pos).normalized().dot(ball_vel) > 1000.0)
      return (ball_vel.y() / ball_vel.x()) * (x - ball_pos.x()) + ball_pos.y();

    // 敵が蹴りそうなら敵の向き
    const auto kicker_itr = std::min_element(
        enemy_robots.cbegin(), enemy_robots.cend(), [&ball_pos](auto& a, auto& b) {
          return (util::math::position(a.second) - ball_pos).norm() <
                 (util::math::position(b.second) - ball_pos).norm();
        });
    if (kicker_itr != enemy_robots.cend()) {
      const auto kicker_pos   = util::math::position(kicker_itr->second);
      const auto kicker_theta = kicker_itr->second.theta();
      if ((kicker_pos - ball_pos).norm() < 1000.0 &&
          std::abs(std::atan2(ball_pos.y() - kicker_pos.y(), ball_pos.x() - kicker_pos.x()) -
                   kicker_theta) < pi<double>() / 3.0)
        return std::tan(kicker_theta) * (x - ball_pos.x()) + ball_pos.y();
    }

    // ボールのy座標
    return ball_pos.y();
  };

  const double x        = wf.x_min() + robot_rad;
  const double y        = std::clamp(f(x + to_kicker_dist), -width / 2.0, width / 2.0);
  const auto keeper_pos = util::math::position(our_robots.at(id_));
  const double dist     = ball_pos.y() - keeper_pos.y();
  double theta          = std::abs(dist) > 100.0 ? std::copysign(pi<double>() / 2, dist) : 0.0;
  if ((Eigen::Vector2d(x, y) - keeper_pos).norm() < 50.0)
    theta = util::math::direction(ball_pos, keeper_pos);
  command.set_position(x, y, theta);
  return command;
}

bool goal_keep::finished() const {
  return false;
}

} // namespace ai_server::game::action
