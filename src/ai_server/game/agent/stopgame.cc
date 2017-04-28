#include <cmath>

#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/util/math.h"

#include "stopgame.h"

namespace ai_server {
namespace game {
namespace agent {

stopgame::stopgame(const model::world& world, bool is_yellow,
                   const std::vector<unsigned int>& ids)
    : base(world, is_yellow), ids_(ids) {
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();
  const double ballx    = ball.x();
  const double bally    = ball.y();

  // 一番ボールに近いロボットを探索し、ボールを追いかけるロボットとする
  double min = 10000000000;
  for (auto id = ids_.begin(); id != ids_.end(); id++) {
    const auto& robot = our_robots.at(*id);
    const double r    = std::hypot(robot.x() - ballx, robot.y() - bally);
    if (r < min) {
      min           = r;
      nearest_robot = *id;
    }
  }
}

std::vector<std::shared_ptr<action::base>> stopgame::execute() {
  std::vector<std::shared_ptr<action::base>> base;
  const auto our_robots      = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball            = world_.ball();
  const double ballx         = ball.x();
  const double bally         = ball.y();
  const double ballxsign     = ballx > 0 ? 1.0 : -1.0;
  const double ballysign     = bally > 0 ? 1.0 : -1.0;
  const double enemygoalsign = world_.field().x_max() > 0 ? 1.0 : -1.0;

  double targetx;
  double targety;

  const double dist = (6000 - std::abs(3000 - std::abs(bally))) / ids_.size();

  int i = 1;
  for (auto id = ids_.begin(); id != ids_.end(); id++) {
    const auto& robot   = our_robots.at(*id);
    const double robotx = robot.x();
    const double roboty = robot.y();

    if (std::abs(ballx) > 2000) {
      // 敵または味方のゴール近く
      if (*id == nearest_robot) {
        // ボールを追いかけるロボット
        targetx = ballx - ballxsign * 650;
        targety = bally;
      } else {
        // それ以外
        targetx = ballx - enemygoalsign * i * 500;
        targety = bally - dist * ballysign * i;
        i++;
      }
    } else {
      // 中間
      if (*id == nearest_robot) {
        // ボールを追いかけるロボット
        targetx = robot.x() > ballx ? ballx + 650 : ballx - 650;
        targety = bally;
      } else {
        // それ以外
        targetx = ballx - enemygoalsign * i * 500;
        targety = bally - dist * ballysign * i;
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

    if (std::hypot(targetx - robotx, targety - roboty) > 100) {
      // 安全な速度にして移動
      const double to_target    = std::atan2(targety - roboty, targetx - robotx);
      targetx                   = robotx + 300 * std::cos(to_target);
      targety                   = roboty + 300 * std::sin(to_target);
      auto move                 = std::make_shared<action::move>(world_, is_yellow_, *id);
      const double to_balltheta = std::atan2(bally - robot.y(), ballx - robot.x());
      move->move_to(targetx, targety, to_balltheta);
      base.push_back(move);
    } else {
      base.push_back(std::make_shared<action::no_operation>(world_, is_yellow_, *id));
    }
  }
  return base;
}
} // agent
} // game
} // ai_server
