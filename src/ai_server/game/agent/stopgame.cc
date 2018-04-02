#include <cmath>
#include <algorithm>
#include <boost/math/constants/constants.hpp>

#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/game/action/vec.h"
#include "ai_server/game/action/autonomous_ball_place.h"
#include "ai_server/util/math.h"
#include "ai_server/util/math/angle.h"

#include "stopgame.h"

using boost::math::constants::pi;

namespace ai_server {
namespace game {
namespace agent {

stopgame::stopgame(const model::world& world, bool is_yellow,
                   const std::vector<unsigned int>& ids)
    : base(world, is_yellow), ids_(ids) {
  nearest_robot_        = 0;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();
  abp_flag_             = false;

  // 一番ボールに近いロボットを探索し、ボールを追いかけるロボットとする
  const auto nearest_robot_id =
      std::min_element(ids_.cbegin(), ids_.cend(), [&ball, &our_robots](auto& a, auto& b) {
        return std::hypot(our_robots.at(a).x() - ball.x(), our_robots.at(a).y() - ball.y()) <
               std::hypot(our_robots.at(b).x() - ball.x(), our_robots.at(b).y() - ball.y());
      });

  if (nearest_robot_id != ids_.end()) {
    nearest_robot_ = *nearest_robot_id;
  }

  abp_ = std::make_shared<action::autonomous_ball_place>(world_, is_yellow_, nearest_robot_,
                                                         abp_target_);
}

// ABP用のコンストラクタ
stopgame::stopgame(const model::world& world, bool is_yellow,
                   const std::vector<unsigned int>& ids, Eigen::Vector2d abp_target)
    : base(world, is_yellow), ids_(ids) {
  nearest_robot_        = 0;
  abp_flag_             = true;
  abp_target_           = abp_target;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();

  // 一番ボールに近いボールを運べるロボットを探索し、ボールを追いかけるロボットとする
  std::vector<unsigned int> placer_ids_{3, 5};
  const auto nearest_robot_id = std::min_element(
      placer_ids_.cbegin(), placer_ids_.cend(), [&ball, &our_robots](auto& a, auto& b) {
        return std::hypot(our_robots.at(a).x() - ball.x(), our_robots.at(a).y() - ball.y()) <
               std::hypot(our_robots.at(b).x() - ball.x(), our_robots.at(b).y() - ball.y());
      });
  if (nearest_robot_id != ids_.end()) {
    nearest_robot_ = *nearest_robot_id;
  }
  abp_ = std::make_shared<action::autonomous_ball_place>(world_, is_yellow_, nearest_robot_,
                                                         abp_target_);
}

std::vector<std::shared_ptr<action::base>> stopgame::execute() {
  std::vector<std::shared_ptr<action::base>> baseaction;
  if (abp_flag_) {
    abp_flag_ = !(abp_->finished());
  }
  if (ids_.size() == 0) {
    return baseaction;
  }
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if (std::any_of(ids_.cbegin(), ids_.cend(),
                  [&our_robots](auto&& id) { return !our_robots.count(id); })) {
    return baseaction;
  }
  const auto ball            = world_.ball();
  const double ballx         = abp_flag_ ? abp_target_.x() : ball.x();
  const double bally         = abp_flag_ ? abp_target_.y() : ball.y();
  const double ballxsign     = ((ballx > 0) || (std::abs(ballx) < 250)) ? 1.0 : -1.0;
  const double ballysign     = ((bally > 0) || (std::abs(bally) < 250)) ? 1.0 : -1.0;
  const double enemygoalsign = world_.field().x_max() > 0 ? 1.0 : -1.0;

  const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  std::vector<unsigned int> enemies_id;
  for (auto& enemy : enemy_robots) {
    enemies_id.push_back(enemy.first);
  }

  double targetx;
  double targety;

  const double dist = (6000 - std::abs(3000 - std::abs(bally))) / ids_.size();

  int i = 2;
  for (auto id : ids_) {
    const auto& robot   = our_robots.at(id);
    const double robotx = robot.x();
    const double roboty = robot.y();

    unsigned int nearest_enemy;
    const auto nearest_enemy_id =
        std::min_element(enemies_id.cbegin(), enemies_id.cend(), [&](auto& a, auto& b) {
          return std::hypot(enemy_robots.at(a).x() - robotx, enemy_robots.at(a).y() - roboty) <
                 std::hypot(enemy_robots.at(b).x() - robotx, enemy_robots.at(b).y() - roboty);
        });
    if (nearest_enemy_id != enemies_id.end()) {
      nearest_enemy = *nearest_enemy_id;
    }
    // 避ける対象
    const auto& it = abp_flag_ ? our_robots.at(nearest_robot_) : enemy_robots.at(nearest_enemy);

    if (std::abs(ballx) > 2000) {
      // 敵または味方のゴール近く
      if (id == nearest_robot_) {
        targetx = ballx - enemygoalsign * 650;
        targety = bally;
      } else {
        // それ以外
        targetx = ballx - enemygoalsign * i * 500;
        targety = bally - dist * ballysign * (i % 2 == 0 ? i : -i) / 2;
        if (std::abs(targety) > 3000) targety = bally - dist * ballysign * (i - 1);
        i++;
      }
    } else {
      // 中間
      if (id == nearest_robot_) {
        targetx = ballx - 650;
        targety = bally;
      } else {
        // それ以外
        targetx = ballx - enemygoalsign * i * 500;
        targety = bally - dist * ballysign * (i % 2 == 0 ? -i : i) / 2;
        if (std::abs(targety) > 3000) targety = bally - dist * ballysign * (i - 1);
        i++;
      }
    }

    if (std::hypot(ballx - robotx, bally - roboty) < 510) {
      // ボールに近かったら遠ざかる
      const double to_robot = std::atan2(roboty - bally, robotx - ballx);
      targetx               = robotx + 200 * std::cos(to_robot);
      targety               = roboty + 200 * std::sin(to_robot);
    } else if (std::hypot(ballx - robotx, bally - roboty) < 520) {
      const double to_targettheta = std::atan2(targety - bally, targetx - ballx);
      const double to_robottheta  = std::atan2(roboty - bally, robotx - ballx);
      const double theta          = util::wrap_to_pi(to_targettheta - to_robottheta);

      if (theta > 0 && std::abs(theta) > 0.3) {
        targetx = robotx - 200 * std::sin(to_robottheta);
        targety = roboty + 200 * std::cos(to_robottheta);
      } else {
        targetx = robotx + 200 * std::sin(to_robottheta);
        targety = roboty - 200 * std::cos(to_robottheta);
      }
    } else if (std::abs(targetx) > 3000 && std::abs(roboty) < 1500) {
      targetx = ballxsign * 2900;
    } else if (std::abs(targetx) > 4200) {
      targetx = ballxsign * 4200;
    } else if (std::abs(robotx) > 3000 && std::abs(roboty) < 1500) {
      targetx = ballxsign * 2900;
    }

    if (id == nearest_robot_ && abp_flag_) {
      // Autonomous Ball Placement
      baseaction.push_back(abp_);
    } else if (std::hypot(it.x() - robotx, it.y() - roboty) < 500.0) {
      // Autonomous Ball Placementをするロボットを避ける
      double vx, vy;
      const double p_to_atheta = util::math::wrap_to_2pi(
          std::atan2(abp_target_.y() - it.y(), abp_target_.x() - it.x()));
      const double p_to_rtheta =
          util::math::wrap_to_2pi(std::atan2(roboty - it.y(), robotx - it.x()));
      const double c_to_ptheta = util::math::wrap_to_2pi(std::atan2(it.y(), it.x()));
      const double c_to_rtheta = util::math::wrap_to_2pi(std::atan2(roboty, robotx));
      if (std::abs(p_to_atheta - p_to_rtheta) < 3.0 * 3.0 * 3.0 * pi<double>() / 4.0) {
        vx = 1000 * (robotx - it.x()) / std::hypot(it.x() - robotx, it.y() - roboty);
        vy = 1000 * (roboty - it.y()) / std::hypot(it.x() - robotx, it.y() - roboty);
      } else {
        vx = 1000 * std::sin(p_to_rtheta) *
             ((c_to_ptheta > c_to_rtheta) - (c_to_ptheta < c_to_rtheta));
        vy = -1000 * std::cos(p_to_rtheta) *
             ((c_to_ptheta > c_to_rtheta) - (c_to_ptheta < c_to_rtheta));
      }
      auto vec = std::make_shared<action::vec>(world_, is_yellow_, id);
      vec->move_to(vx, vy, 0.0);
      baseaction.push_back(vec);
    } else if (std::hypot(targetx - robotx, targety - roboty) > 100) {
      auto move                 = std::make_shared<action::move>(world_, is_yellow_, id);
      const double to_balltheta = std::atan2(bally - robot.y(), ballx - robot.x());
      move->move_to(targetx, targety, to_balltheta);
      baseaction.push_back(move);
    } else {
      baseaction.push_back(std::make_shared<action::no_operation>(world_, is_yellow_, id));
    }
  }
  return baseaction;
}
} // namespace agent
} // namespace game
} // namespace ai_server
