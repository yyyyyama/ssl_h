#include <cmath>
#include <algorithm>
#include <boost/math/constants/constants.hpp>

#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/game/action/receive.h"
#include "ai_server/game/action/vec.h"
#include "ai_server/game/action/autonomous_ball_place.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/util/math/angle.h"

#include "stopgame.h"

using boost::math::constants::pi;
using namespace std::chrono_literals;

namespace ai_server {
namespace game {
namespace agent {

// 通常stopgame用コンストラクタ
stopgame::stopgame(const model::world& world, bool is_yellow,
                   const std::vector<unsigned int>& ids)
    : base(world, is_yellow), ids_(ids) {
  nearest_robot_        = 0;
  receiver_robot_       = 1;
  is_placement_         = false;
  abp_flag_             = false;
  abp_target_           = Eigen::Vector2d(0, 0);
  kick_abp_flag_        = false;
  abp_start_            = false;
  wait_flag_            = false;
  before_kick_finished_ = false;
  kick_failed_          = false;
  pass_dist_            = 0.0;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();
  begin_dist_           = 0.0;
  Eigen::Vector2d receiverxy_{0, 0};

  std::vector<unsigned int> placer_ids;
  for (auto id : ids_) {
    if (our_robots.count(id)) placer_ids.push_back(id);
  }

  if (placer_ids.size() >= 2) {
    const auto nearest_robot_id = std::min_element(
        placer_ids.cbegin(), placer_ids.cend(), [&ball, &our_robots](auto& a, auto& b) {
          return std::hypot(our_robots.at(a).x() - ball.x(), our_robots.at(a).y() - ball.y()) <
                 std::hypot(our_robots.at(b).x() - ball.x(), our_robots.at(b).y() - ball.y());
        });
    if (nearest_robot_id != ids_.end()) {
      nearest_robot_ = *nearest_robot_id;
    }
    placer_ids.erase(nearest_robot_id);
    const auto receiver_robot_id =
        std::min_element(placer_ids.cbegin(), placer_ids.cend(), [&](auto& a, auto& b) {
          return std::hypot(our_robots.at(a).x() - abp_target_.x(),
                            our_robots.at(a).y() - abp_target_.y()) <
                 std::hypot(our_robots.at(b).x() - abp_target_.x(),
                            our_robots.at(b).y() - abp_target_.y());
        });
    if (receiver_robot_id != ids_.end()) {
      receiver_robot_ = *receiver_robot_id;
    }

    kick_    = std::make_shared<action::kick_action>(world_, is_yellow_, nearest_robot_);
    receive_ = std::make_shared<action::receive>(world_, is_yellow_, nearest_robot_);
    abp_  = std::make_shared<action::autonomous_ball_place>(world_, is_yellow_, nearest_robot_,
                                                           abp_target_);
    pull_ = std::make_shared<action::autonomous_ball_place>(world_, is_yellow_, nearest_robot_,
                                                            abp_target_);
  } else {
    std::vector<unsigned int> placer_ids = ids_;
    const auto nearest_robot_id          = std::min_element(
        placer_ids.cbegin(), placer_ids.cend(), [&ball, &our_robots](auto& a, auto& b) {
          return std::hypot(our_robots.at(a).x() - ball.x(), our_robots.at(a).y() - ball.y()) <
                 std::hypot(our_robots.at(b).x() - ball.x(), our_robots.at(b).y() - ball.y());
        });
    if (nearest_robot_id != ids_.end()) {
      nearest_robot_ = *nearest_robot_id;
    }
    kick_    = std::make_shared<action::kick_action>(world_, is_yellow_, nearest_robot_);
    receive_ = std::make_shared<action::receive>(world_, is_yellow_, receiver_robot_);
    abp_  = std::make_shared<action::autonomous_ball_place>(world_, is_yellow_, nearest_robot_,
                                                           abp_target_);
    pull_ = std::make_shared<action::autonomous_ball_place>(world_, is_yellow_, nearest_robot_,
                                                            abp_target_);
  }
}

// ABP用コンストラクタ
stopgame::stopgame(const model::world& world, bool is_yellow,
                   const std::vector<unsigned int>& ids, Eigen::Vector2d abp_target,
                   bool abp_flag)
    : base(world, is_yellow), ids_(ids) {
  nearest_robot_        = 0;
  receiver_robot_       = 1;
  is_placement_         = true;
  abp_flag_             = abp_flag;
  abp_target_           = abp_target;
  kick_abp_flag_        = false;
  abp_start_            = false;
  wait_flag_            = true;
  before_kick_finished_ = false;
  kick_failed_          = false;
  // これ以上の距離ならパス
  pass_dist_            = 5000.0;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();
  begin_dist_           = std::hypot(abp_target_.x() - ball.x(), abp_target_.y() - ball.y());
  Eigen::Vector2d receiverxy_{0, 0};

  std::vector<unsigned int> init_ids_;
  for (auto id : ids_) {
    if (our_robots.count(id)) init_ids_.push_back(id);
  }
  if (init_ids_.size() >= 2) {
    // パスでabp
    const auto receiver_robot_id =
        std::min_element(init_ids_.cbegin(), init_ids_.cend(), [&](auto& a, auto& b) {
          return std::hypot(our_robots.at(a).x() - abp_target_.x(),
                            our_robots.at(a).y() - abp_target_.y()) <
                 std::hypot(our_robots.at(b).x() - abp_target_.x(),
                            our_robots.at(b).y() - abp_target_.y());
        });
    if (receiver_robot_id != init_ids_.end()) {
      receiver_robot_ = *receiver_robot_id;
    }
    init_ids_.erase(receiver_robot_id);

    const auto nearest_robot_id = std::min_element(
        init_ids_.cbegin(), init_ids_.cend(), [&ball, &our_robots](auto& a, auto& b) {
          return std::hypot(our_robots.at(a).x() - ball.x(), our_robots.at(a).y() - ball.y()) <
                 std::hypot(our_robots.at(b).x() - ball.x(), our_robots.at(b).y() - ball.y());
        });
    if (nearest_robot_id != init_ids_.end()) {
      nearest_robot_ = *nearest_robot_id;
    }
    kick_    = std::make_shared<action::kick_action>(world_, is_yellow_, nearest_robot_);
    receive_ = std::make_shared<action::receive>(world_, is_yellow_, receiver_robot_);
    abp_  = std::make_shared<action::autonomous_ball_place>(world_, is_yellow_, receiver_robot_,
                                                           abp_target_);
    pull_ = std::make_shared<action::autonomous_ball_place>(world_, is_yellow_, nearest_robot_,
                                                            abp_target_);
  } else {
    // 単独でabp
    std::vector<unsigned int> init_ids_ = ids_;
    const auto nearest_robot_id         = std::min_element(
        init_ids_.cbegin(), init_ids_.cend(), [&ball, &our_robots](auto& a, auto& b) {
          return std::hypot(our_robots.at(a).x() - ball.x(), our_robots.at(a).y() - ball.y()) <
                 std::hypot(our_robots.at(b).x() - ball.x(), our_robots.at(b).y() - ball.y());
        });
    if (nearest_robot_id != init_ids_.end()) {
      nearest_robot_ = *nearest_robot_id;
    }
    kick_    = std::make_shared<action::kick_action>(world_, is_yellow_, nearest_robot_);
    receive_ = std::make_shared<action::receive>(world_, is_yellow_, receiver_robot_);
    abp_  = std::make_shared<action::autonomous_ball_place>(world_, is_yellow_, nearest_robot_,
                                                           abp_target_);
    pull_ = std::make_shared<action::autonomous_ball_place>(world_, is_yellow_, nearest_robot_,
                                                            abp_target_);
  }
}

std::vector<std::shared_ptr<action::base>> stopgame::execute() {
  std::vector<std::shared_ptr<action::base>> baseaction;

  std::vector<unsigned int> visible_ids;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  for (auto id : ids_) {
    if (our_robots.count(id)) visible_ids.push_back(id);
  }
  if (visible_ids.size() == 0) {
    return baseaction;
  }
  if (abp_flag_) {
    abp_flag_ = !(abp_->finished());
    if (kick_failed_) abp_flag_ = !(pull_->finished());
    if (ids_.size() >= 2 && !abp_flag_) {
      // パスによるabp終了時、受け取り側のロボットをボールに最も近いロボットと設定しなおす
      nearest_robot_ = receiver_robot_;
    }
  }
  const auto ball            = world_.ball();
  const double ballx         = is_placement_ ? abp_target_.x() : ball.x();
  const double bally         = is_placement_ ? abp_target_.y() : ball.y();
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
  const double dist =
      (2 * world_.field().y_max() - std::abs(world_.field().y_max() - std::abs(bally))) /
      ids_.size();

  if (init_ids_.size() >= 2) {
    receiverxy_ =
        Eigen::Vector2d(our_robots.at(receiver_robot_).x(), our_robots.at(receiver_robot_).y());
  }

  {
    // パスでのABP関連の処理
    kick_abp_flag_ =
        init_ids_.size() >= 2 && !kick_->finished() &&
        std::hypot(abp_target_.x() - ball.x(), abp_target_.y() - ball.y()) > pass_dist_;

    if (visible_ids.size() >= 2 && !before_kick_finished_ && kick_->finished())
      kick_time_ = util::clock_type::now();
    if (visible_ids.size() >= 2 &&
        std::hypot(abp_target_.x() - ball.x(), abp_target_.y() - ball.y()) >
            std::abs(begin_dist_ - 1000) &&
        kick_->finished() && (util::clock_type::now() - kick_time_) > 1s) {
      // キック失敗時、キック予定のロボットがabpするように変更
      kick_failed_ = true;
    }
    before_kick_finished_ = kick_->finished();
  }

  int i = 2;
  for (auto id : visible_ids) {
    if (our_robots.count(id)) {
      const auto& robot   = our_robots.at(id);
      const double robotx = robot.x();
      const double roboty = robot.y();

      // ロボットに一番近い敵ロボット抽出
      unsigned int nearest_enemy;
      const auto nearest_enemy_id =
          std::min_element(enemies_id.cbegin(), enemies_id.cend(), [&](auto& a, auto& b) {
            return std::hypot(enemy_robots.at(a).x() - robotx,
                              enemy_robots.at(a).y() - roboty) <
                   std::hypot(enemy_robots.at(b).x() - robotx, enemy_robots.at(b).y() - roboty);
          });
      if (nearest_enemy_id != enemies_id.end()) {
        nearest_enemy = *nearest_enemy_id;
      }
      // 避ける対象
      auto& abp_robot = abp_flag_ && our_robots.count(nearest_robot_)
                            ? our_robots.at(nearest_robot_)
                            : enemy_robots.at(nearest_enemy);
      Eigen::Vector2d it{abp_robot.x(), abp_robot.y()};
      if (abp_flag_ && init_ids_.size() >= 2 && !kick_abp_flag_) {
        it = Eigen::Vector2d(our_robots.at(receiver_robot_).x(),
                             our_robots.at(receiver_robot_).y());
      }

      if (std::abs(ballx) > world_.field().x_max() - 2500) {
        // 敵または味方のゴール近く
        if (id == nearest_robot_) {
          if (is_placement_) {
            const double to_robot = std::atan2(roboty - bally, robotx - ballx);
            targetx               = ballx + 650 * std::cos(to_robot);
            targety               = bally + 650 * std::sin(to_robot);
          } else {
            targetx = ballx - enemygoalsign * 650;
            targety = bally;
          }
        } else {
          // それ以外
          targetx = ballx - enemygoalsign * i * 500;
          targety = bally - dist * ballysign * (i % 2 == 0 ? i : -i) / 2;
          if (std::abs(targety) > world_.field().y_max())
            targety = bally - dist * ballysign * (i - 1);
          i++;
        }
      } else {
        // 中間
        if (id == nearest_robot_) {
          if (is_placement_) {
            const double to_robot = std::atan2(roboty - bally, robotx - ballx);
            targetx               = ballx + 650 * std::cos(to_robot);
            targety               = bally + 650 * std::sin(to_robot);
          } else {
            targetx = ballx - 650;
            targety = bally;
          }
        } else {
          // それ以外
          targetx = ballx - enemygoalsign * i * 500;
          targety = bally - dist * ballysign * (i % 2 == 0 ? -i : i) / 2;
          if (std::abs(targety) > world_.field().y_max())
            targety = bally - dist * ballysign * (i - 1);
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
        const double theta          = util::math::wrap_to_pi(to_targettheta - to_robottheta);

        if (theta > 0 && std::abs(theta) > 0.3) {
          targetx = robotx - 200 * std::sin(to_robottheta);
          targety = roboty + 200 * std::cos(to_robottheta);
        } else {
          targetx = robotx + 200 * std::sin(to_robottheta);
          targety = roboty - 200 * std::cos(to_robottheta);
        }
      } else if (std::abs(targetx) > world_.field().x_max() - 1500 && std::abs(roboty) < 1500) {
        targetx = ballxsign * (world_.field().x_max() - 1600);
      } else if (std::abs(targetx) > world_.field().x_max() - 300) {
        targetx = ballxsign * (world_.field().x_max() - 300);
      } else if (std::abs(robotx) > world_.field().x_max() - 1500 && std::abs(roboty) < 1500) {
        targetx = ballxsign * (world_.field().x_max() - 1600);
      }

      if (kick_abp_flag_ && id == nearest_robot_ && is_placement_ && abp_flag_ &&
          kick_failed_) {
        baseaction.push_back(pull_);
      } else if (visible_ids.size() >= 2 && id == nearest_robot_ && abp_flag_ &&
                 kick_abp_flag_) {
        // Autonomous Ball Placement kicker
        if (std::hypot(receiverxy_.x() - abp_target_.x(), receiverxy_.y() - abp_target_.y()) <
                200.0 &&
            std::abs(ball.x()) < world_.field().x_max() &&
            std::abs(ball.y()) < world_.field().y_max()) {
          kick_->set_mode(action::kick_action::mode::goal);
          // receiverに向かってキック
          // kick_->kick_to(abp_target_.x(), abp_target_.y());
          kick_->kick_to(receiverxy_.x(), receiverxy_.y());
          kick_->set_kick_type({model::command::kick_type_t::line, 50});
          kick_->set_angle_margin(0.10);
          kick_->set_stop_ball(true);
          baseaction.push_back(kick_);
        } else if (std::abs(ball.x()) > world_.field().x_max() ||
                   std::abs(ball.y()) > world_.field().y_max()) {
          // 壁際にボールがあれば引っ張ってくる
          baseaction.push_back(pull_);
        } else {
          /*
          auto vec = std::make_shared<action::vec>(world_, is_yellow_, id);
          vec->move_to(0.0, 0.0, 0.0);
          baseaction.push_back(vec);
          */

          kick_->set_mode(action::kick_action::mode::goal);
          kick_->kick_to(abp_target_.x(), abp_target_.y());
          kick_->set_kick_type({model::command::kick_type_t::none, 50});
          kick_->set_dribble(2);
          kick_->set_angle_margin(0.10);
          kick_->set_stop_ball(true);
          baseaction.push_back(kick_);
        }
      } else if (kick_abp_flag_ && id == receiver_robot_ && is_placement_ && abp_flag_ &&
                 !kick_failed_) {
        // Autonomous Ball Placement receiver
        const double to_nrtheta = util::math::wrap_to_2pi(
            std::atan2(abp_target_.y() - ball.y(), abp_target_.x() - ball.x()));
        const double receive_theta = util::math::wrap_to_2pi(
            std::atan2(abp_target_.y() - roboty, abp_target_.x() - robotx));
        const double r_x = abp_target_.x() + 120 * std::cos(to_nrtheta);
        const double r_y = abp_target_.y() + 120 * std::sin(to_nrtheta);
        if (kick_abp_flag_ &&
            !(kick_->state() == action::kick_action::running_state::finished)) {
          auto move = std::make_shared<action::move>(world_, is_yellow_, id);
          move->move_to(r_x, r_y, receive_theta);
          baseaction.push_back(move);
        } else {
          if (wait_flag_ && kick_->finished()) {
            begin_     = util::clock_type::now();
            wait_flag_ = false;
          }
          now_ = util::clock_type::now();
          if ((!wait_flag_ && (now_ - begin_) > 4s) ||
              std::abs(ball.x()) > world_.field().x_max() ||
              std::abs(ball.y()) > world_.field().y_max() || abp_start_ ||
              receive_->finished() ||
              (std::hypot(ball.vx(), ball.vy()) < 100.0 &&
               (std::hypot(abp_target_.x() - ball.x(), abp_target_.y() - ball.y()) <
                pass_dist_))) {
            abp_start_ = true;
            baseaction.push_back(abp_);
          } else {
            receive_->set_passer(nearest_robot_);
            receive_->set_dribble(5);
            baseaction.push_back(receive_);
          }
        }
      } else if (!kick_abp_flag_ && id == nearest_robot_ && is_placement_ && abp_flag_) {
        // 単独でABP
        baseaction.push_back(pull_);
      } else if (is_placement_ && our_robots.at(id).x() != it.x() &&
                 std::hypot(it.x() - robotx, it.y() - roboty) < 500.0) {
        // Autonomous Ball Placementをするロボットを避ける
        double vx, vy;
        const double c_to_ptheta = util::math::wrap_to_2pi(std::atan2(it.y(), it.x()));
        const double c_to_rtheta = util::math::wrap_to_2pi(std::atan2(roboty, robotx));
        if (std::abs(robot.x()) < world_.field().x_max() - 750.0 &&
            std::abs(robot.y()) < world_.field().y_max() - 750.0) {
          vx = 1000 * (robotx - it.x()) / std::hypot(it.x() - robotx, it.y() - roboty);
          vy = 1000 * (roboty - it.y()) / std::hypot(it.x() - robotx, it.y() - roboty);
        } else {
          vx = 1000 * std::sin(c_to_rtheta) *
               ((c_to_ptheta > c_to_rtheta) - (c_to_ptheta < c_to_rtheta));
          vy = -1000 * std::cos(c_to_rtheta) *
               ((c_to_ptheta > c_to_rtheta) - (c_to_ptheta < c_to_rtheta));
        }
        /*
        if (id == nearest_robot_ && abp_flag_) {
          vx = 0.0;
          vy = 0.0;
        }
        */
        auto vec = std::make_shared<action::vec>(world_, is_yellow_, id);
        vec->move_to(vx, vy, 0.0);
        baseaction.push_back(vec);
      } else if (is_placement_ &&
                 std::abs(
                     std::hypot(ball.x() - robot.x(), ball.y() - robot.y()) *
                     std::sin(util::math::wrap_to_2pi(
                         std::atan2(abp_target_.y() - ball.y(), abp_target_.x() - ball.x()) -
                         std::atan2(abp_target_.y() - robot.y(),
                                    abp_target_.x() - robot.x())))) < 250.0) {
        // ABP軌道上にいないようにする
        if (std::abs(std::hypot(ball.x() - robot.x(), ball.y() - robot.y()) *
                     std::sin(util::math::wrap_to_2pi(
                         std::atan2(abp_target_.y() - ball.y(), abp_target_.x() - ball.x()) -
                         std::atan2(abp_target_.y() - robot.y(),
                                    abp_target_.x() - robot.x())))) < 150.0) {
          // ABP軌道から離れる
          const double leave_theta = util::math::wrap_to_2pi(
              std::atan2(abp_target_.y() - ball.y(), abp_target_.x() - ball.x()) +
              pi<double>() / 2.0);
          const double x = robot.x() + 200 * std::cos(leave_theta);
          const double y = robot.y() + 200 * std::sin(leave_theta);
          auto move      = std::make_shared<action::move>(world_, is_yellow_, id);
          move->move_to(x, y, robot.theta());
          baseaction.push_back(move);
        } else {
          //停止する
          auto vec = std::make_shared<action::vec>(world_, is_yellow_, id);
          vec->move_to(0.0, 0.0, 0.0);
          baseaction.push_back(vec);
        }
      } else if (std::hypot(targetx - robotx, targety - roboty) > 100 ||
                 (util::math::wrap_to_2pi(std::atan2(bally - robot.y(), ballx - robot.x())) -
                  robot.theta()) > pi<double>() / 8.0) {
        auto move = std::make_shared<action::move>(world_, is_yellow_, id);
        const double to_balltheta =
            util::math::wrap_to_2pi(std::atan2(bally - robot.y(), ballx - robot.x()));
        move->move_to(targetx, targety, to_balltheta);
        baseaction.push_back(move);
      } else {
        baseaction.push_back(std::make_shared<action::no_operation>(world_, is_yellow_, id));
      }
    }
  }
  return baseaction;
}
} // namespace agent
} // namespace game
} // namespace ai_server
