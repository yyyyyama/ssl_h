#include <Eigen/Geometry>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/model/command.h"
#include "ai_server/util/math.h"

#include "setplay.h"

#include <iostream>

namespace ai_server {
namespace game {
namespace agent {

setplay::setplay(const model::world& world, bool is_yellow, unsigned int kicker_id,
                 const std::vector<unsigned int>& receiver_ids)
    : base(world, is_yellow), kicker_id_(kicker_id), receiver_ids_(receiver_ids) {
  kick_ = make_action<action::kick_action>(kicker_id_);
}

std::vector<std::shared_ptr<action::base>> setplay::execute() {
  using boost::math::constants::pi;
  std::vector<std::shared_ptr<action::base>> baseaction;
  const auto our_robots  = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto enemyrobots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  const auto& kicker     = our_robots.at(kicker_id_);

  const auto ball        = world_.ball();
  const double ballysign = ball.y() > 0 ? 1.0 : -1.0;

  // const double enemygoal     = world_.field().x_max();
  // const double enemygoalsign = enemygoal > 0 ? 1.0 : -1.0;
  const double enemygoal     = 4500;
  const double enemygoalsign = enemygoal > 0 ? 1.0 : -1.0;

  const auto old_ball = ball_log_.front();

  const Eigen::Vector2d passpos(enemygoalsign * 1000, 0);

  // /* 敵の密集度を行列で評価する このデータ使ってパス位置導出したい */
  // const Eigen::Vector2i split(18, 12);
  // const Eigen::Vector2i gap(9000 / split.x(), 6000 / split.y());
  // Eigen::MatrixXi enemydata = Eigen::MatrixXi::Zero(split.x(), split.y());
  // for (auto enemyrobot_p : enemyrobots) {
  //   const auto enemyrobot = std::get<1>(enemyrobot_p);
  //   const Eigen::Vector2i point((enemyrobot.x() + 4500) / gap.x(),
  //                               (enemyrobot.y() + 3000) / gap.y());
  //   for (int i = -1; i <= 1; i++) {
  //     for (int j = -1; j <= 1; j++) {
  //       const Eigen::Vector2i target(point.x() + i, point.y() + j);
  //       if (i == 0 && j == 0) {
  //         enemydata(point.x(), point.y()) += 3;
  //       } else if (-1 < target.x() && -1 < target.y() && target.x() < split.x() &&
  //                  target.y() < split.y()) {
  //         enemydata(target.x(), target.y()) += 1;
  //       }
  //     }
  //   }
  // }

  // std::cout << std::endl;
  // for (int j = split.y() - 1; j >= 0; j--) {
  //   for (int i = 0; i < split.x(); i++) {
  //     std::cout << enemydata(i, j);
  //   }
  //   std::cout << std::endl;
  // }

  if (receiver_ids_.size() == 0) {
    if (kick_->finished()||state_ == state::finished) {
      // std::cout << "fini" << std::endl;
      state_ = state::finished;
          baseaction.push_back(make_action<action::no_operation>(kicker_id_));
    } else {
      double max_dist = 0;
      double target;

      std::vector<double> goalpoint{-270, 0, 270};
      for (auto point : goalpoint) {
        double min_dist = 100000;
        for (auto enemyrobot_p : enemyrobots) {
          auto enemyrobot = std::get<1>(enemyrobot_p);
          if ((ball.x() - enemyrobot.x()) * (enemygoal - enemyrobot.x() < 0)) {
            const Eigen::Vector3d to_point(enemygoal - ball.x(), point - ball.y(), 0);
            const Eigen::Vector3d to_enemy(enemyrobot.x() - ball.x(), enemyrobot.y() - ball.y(),
                                           0);
            const double distance = std::abs(to_point.cross(to_enemy).norm() / to_point.norm());
            if (distance < min_dist) {
              min_dist = distance;
            }
          }
        }
        if (min_dist > max_dist) {
          max_dist = min_dist;
          target   = point;
        }
      }

      const double power = 100;
      // const double power = 5;

      kick_->kick_to(enemygoal, target);
      kick_->set_mode(action::kick_action::mode::ball);
      kick_->set_anglemargin(0.02);
      kick_->set_kick_type({model::command::kick_type_t::line, power});
      baseaction.push_back(kick_);
    }

  } else {
    if (kick_->finished()) {
      if (state_ == state::pass) {
        kick_  = make_action<action::kick_action>(shooter_id_);
        state_ = state::shoot;
      } else {
        state_ = state::finished;
      }
    }

    switch (state_) {
      case setplay::state::setup: {
        // // std::cout << "setup" << std::endl;
        if (positions.size() == 0) {
          /* いいかんじのばしょをしていしたかった */
          int i = 0;
          for (auto receiver_id : receiver_ids_) {
            const auto& robot  = our_robots.at(receiver_id);
            const double rad   = -0.4;
            const double theta = ballysign * (i / 2 + 1) * rad * (1.0 - 2 * (i % 2));
            const double dire  = std::atan2(ball.y() - robot.y(), ball.x() - robot.x());
            if (std::abs(ball.x() - enemygoal) < 3000) {
              pos_ = pos::far;
              positions.push_back({enemygoal - enemygoalsign * std::cos(theta) * 2000,
                                   std::sin(theta) * 2000, dire});
            } else if (std::abs(ball.x() - enemygoal) > 6000) {
              pos_ = pos::near;
              positions.push_back(
                  {ball.x() + enemygoalsign * 1500, std::sin(theta) * 2000, dire});
            } else {
              pos_ = pos::mid;
              positions.push_back(
                  {ball.x() + enemygoalsign * i * 1000, std::sin(theta) * 2000, dire});
            }
            i++;
          }
        }
        /* 指定位置に動いたか確認できたらshooter_id_を決定し、stateを次の状態にする */
        int i          = 0;
        bool movedflag = true;
        for (auto receiver_id : receiver_ids_) {
          if (std::hypot(positions[i].x() - our_robots.at(receiver_id).x(),
                         positions[i].y() - our_robots.at(receiver_id).y()) > 100) {
            movedflag = false;
            break;
          }
          i++;
        }
        if (std::hypot(ball.x() - kicker.x(), ball.y() - kicker.y()) > 2000) {
          movedflag = false;
        }
        if (movedflag) {
          state_          = state::pass;
          double max_dist = 0;
          for (auto receiver_id : receiver_ids_) {
            double min_dist = 100000;
            auto& receiver  = our_robots.at(receiver_id);
            for (auto enemyrobot_p : enemyrobots) {
              auto enemyrobot = std::get<1>(enemyrobot_p);
              if ((ball.x() - enemyrobot.x()) * (receiver.x() - enemyrobot.x() < 0)) {
                Eigen::Vector3d to_shooter(receiver.x() - ball.x(), receiver.y() - ball.y(), 0);
                Eigen::Vector3d to_enemy(enemyrobot.x() - ball.x(), enemyrobot.y() - ball.y(),
                                         0);
                const double distance =
                    std::abs(to_shooter.cross(to_enemy).norm() / to_shooter.norm());
                if (distance < min_dist) {
                  min_dist = distance;
                }
              }
            }
            if (min_dist > max_dist) {
              max_dist    = min_dist;
              shooter_id_ = receiver_id;
            }
          }
        } else {
          for (i = 0; i < static_cast<double>(receiver_ids_.size()); i++) {
            const auto& robot = our_robots.at(receiver_ids_[i]);
            auto move         = make_action<action::move>(receiver_ids_[i]);
            const double to_target =
                std::atan2(positions[i].y() - robot.y(), positions[i].x() - robot.x());
			std::cout<<"ss1"<<std::endl;
            if (std::hypot(positions[i].x() - robot.x(), positions[i].y() - robot.y()) < 50) {
              move->move_to(robot.x(), robot.y(),
                            std::atan2(ball.y() - robot.y(), ball.x() - robot.x()));
            } else {
              move->move_to(robot.x() + 600 * std::cos(to_target),
                            robot.y() + 600 * std::sin(to_target),
                            std::atan2(ball.y() - robot.y(), ball.x() - robot.x()));
            }
            baseaction.push_back(move);
          }

          if (std::hypot(ball.x() - kicker.x(), ball.y() - kicker.y()) > 500) {
			std::cout<<"ss2"<<std::endl;
            auto move            = make_action<action::move>(kicker_id_);
            const double to_ball = std::atan2(ball.y() - kicker.y(), ball.x() - kicker.x());
            move->move_to(kicker.x() + 500 * std::cos(to_ball),
                          kicker.y() + 500 * std::sin(to_ball), to_ball);
            baseaction.push_back(move);
          } else {
            baseaction.push_back(make_action<action::no_operation>(kicker_id_));
          }
          break;
        }
      }

      case state::pass: {
        // // std::cout << "pass" << std::endl;
        auto shooter = our_robots.at(shooter_id_);
        kick_->set_mode(action::kick_action::mode::ball);

        bool line_flag = true;

        if (pos_ == pos::near) {
          kick_->kick_to(shooter.x() + enemygoalsign * 100, shooter.y());
          for (auto enemyrobot_p : enemyrobots) {
            auto enemyrobot = std::get<1>(enemyrobot_p);
            Eigen::Vector3d to_shooter(shooter.x() - kicker.x(), shooter.y() - kicker.y(), 0);
            Eigen::Vector3d to_enemy(enemyrobot.x() - kicker.x(), enemyrobot.y() - kicker.y(),
                                     0);
            const double distance =
                std::abs(to_shooter.cross(to_enemy).norm() / to_shooter.norm());
            if (distance < 100) {
              line_flag = false;
              break;
            }
          }
          line_flag = true;
        } else {
          kick_->kick_to(shooter.x(), shooter.y());
          /* 間にロボットがいるか */
          for (auto enemyrobot_p : enemyrobots) {
            auto enemyrobot = std::get<1>(enemyrobot_p);
            Eigen::Vector3d to_shooter(shooter.x() - kicker.x(), shooter.y() - kicker.y(), 0);
            Eigen::Vector3d to_enemy(enemyrobot.x() - kicker.x(), enemyrobot.y() - kicker.y(),
                                     0);
            const double distance =
                std::abs(to_shooter.cross(to_enemy).norm() / to_shooter.norm());
            if (distance < 100) {
              line_flag = false;
              break;
            }
          }
        }
        // line_flag = true;

        if (line_flag) {
          const double power = 30;
          // const double power = 5;

          kick_->set_dribble(4);
          kick_->set_anglemargin(0.01);
          kick_->set_kick_type({model::command::kick_type_t::line, power});
        } else {
          /* ここで距離とパワーの変換関数を入れたい */
          const double power = 150;
          // const double power = 10;

          kick_->set_dribble(4);
          kick_->set_anglemargin(0.01);
          kick_->set_kick_type({model::command::kick_type_t::chip, power});
        }
        baseaction.push_back(kick_);

        for (auto receiver_id : receiver_ids_) {
          const auto receiver = our_robots.at(receiver_id);
          if (pos_ == pos::near) {
            const Eigen::Vector2d kickerball(passpos.x() - old_ball.x(),
                                             passpos.y() - old_ball.y());
            const Eigen::Vector2d to_shooter(receiver.x() - kicker.x(),
                                             receiver.y() - kicker.y());
            const Eigen::Vector2d normaliz = kickerball.normalized();
            const double dot               = normaliz.dot(to_shooter);
            Eigen::Vector2d target(kicker.x() + normaliz.x() * dot,
                                   kicker.y() + normaliz.y() * dot);
            const double theta = std::atan2(normaliz.y(), normaliz.x()) - pi<double>();
            auto move          = make_action<action::move>(shooter_id_);
            move->move_to(target.x(), target.y(), theta);

            baseaction.push_back(move);
          } else {
            /* 暫定moveの場所に移動？してそれをbaseactionに代入 */
            auto move = make_action<action::move>(receiver_id);
            move->move_to(receiver.x(), receiver.y(),
                          std::atan2(ball.y() - receiver.y(), ball.x() - receiver.x()));
            baseaction.push_back(move);
          }
        }
        break;
      }

      case state::shoot: {
        // // std::cout << "shoot" << std::endl;
        baseaction.push_back(make_action<action::no_operation>(kicker_id_));
        for (auto receiver_id : receiver_ids_) {
          /* shooterにkick_actionいれてほかはnoop? */
          const auto receiver = our_robots.at(receiver_id);
          if (std::hypot(enemygoal - receiver.x(), receiver.y()) > 1000 &&
              std::abs(ball.x() < 4500) && std::abs(ball.y()) < 3000) {
            if (std::hypot(receiver.x() - kicker.x(), receiver.y() - kicker.y()) >
                std::hypot(ball.x() - kicker.x(), ball.y() - kicker.y())) {
              if (receiver_id == shooter_id_) {
                if (pos_ == pos::near &&
                    std::hypot(receiver.x() - ball.x(), receiver.y() - ball.y()) > 1000) {
                  const Eigen::Vector2d kickerball(ball.x() - old_ball.x(),
                                                   ball.y() - old_ball.y());
                  const Eigen::Vector2d to_shooter(receiver.x() - kicker.x(),
                                                   receiver.y() - kicker.y());
                  const Eigen::Vector2d normaliz = kickerball.normalized();
                  const double dot               = normaliz.dot(to_shooter);
                  Eigen::Vector2d target(kicker.x() + normaliz.x() * dot,
                                         kicker.y() + normaliz.y() * dot);
                  const double theta = std::atan2(normaliz.y(), normaliz.x()) + pi<double>();
                  auto move          = make_action<action::move>(shooter_id_);
                  // if (target.norm() > 600) {
                  //   target = target.normalized() * 600;
                  // }
                  move->move_to(target.x(), target.y(), theta);

                  baseaction.push_back(move);
                } else {
                  double max_dist = 0;
                  double target;

                  std::vector<double> goalpoint{-270, 0, 270};
                  for (auto point : goalpoint) {
                    double min_dist = 100000;
                    for (auto enemyrobot_p : enemyrobots) {
                      auto enemyrobot = std::get<1>(enemyrobot_p);
                      if ((ball.x() - enemyrobot.x()) * (enemygoal - enemyrobot.x() < 0)) {
                        const Eigen::Vector3d to_point(enemygoal - ball.x(), point - ball.y(),
                                                       0);
                        const Eigen::Vector3d to_enemy(enemyrobot.x() - ball.x(),
                                                       enemyrobot.y() - ball.y(), 0);
                        const double distance =
                            std::abs(to_point.cross(to_enemy).norm() / to_point.norm());
                        if (distance < min_dist) {
                          min_dist = distance;
                        }
                      }
                    }
                    if (min_dist > max_dist) {
                      max_dist = min_dist;
                      target   = point;
                    }
                  }

                  if (std::hypot(ball.x() - receiver.x(), ball.y() - receiver.y()) < 1000) {
                    const double power = 100;
                    // const double power = 5;

                    kick_->kick_to(enemygoal, target);
                    kick_->set_dribble(9);
                    kick_->set_mode(action::kick_action::mode::ball);
                    kick_->set_anglemargin(0.03);
                    kick_->set_kick_type({model::command::kick_type_t::line, power});
                    baseaction.push_back(kick_);
                  } else {
                    // // std::cout << "nyan" << std::endl;
                    const Eigen::Vector2d kickerball(ball.x() - old_ball.x(),
                                                     ball.y() - old_ball.y());
                    const Eigen::Vector2d to_shooter(receiver.x() - kicker.x(),
                                                     receiver.y() - kicker.y());
                    const Eigen::Vector2d normaliz = kickerball.normalized();
                    const double dot               = normaliz.dot(to_shooter);
                    Eigen::Vector2d target(kicker.x() + normaliz.x() * dot,
                                           kicker.y() + normaliz.y() * dot);
                    const double theta = std::atan2(normaliz.y(), normaliz.x()) + pi<double>();
                    auto move          = make_action<action::move>(shooter_id_);
                    move->move_to(target.x(), target.y(), theta);
                    baseaction.push_back(move);
                  }
                }
              } else {

                baseaction.push_back(make_action<action::no_operation>(receiver_id));
              }
            } else {
              const double power = 100;
              // const double power = 5;

              kick_->kick_to(enemygoal, 0);
              kick_->set_dribble(9);
              kick_->set_mode(action::kick_action::mode::ball);
              kick_->set_anglemargin(0.03);
              kick_->set_kick_type({model::command::kick_type_t::line, power});
              baseaction.push_back(kick_);
            }
          } else {
            baseaction.push_back(make_action<action::no_operation>(receiver_id));
          }
        }
        break;
      }

      case state::finished: {
        // // std::cout << "finished" << std::endl;
        baseaction.push_back(make_action<action::no_operation>(kicker_id_));
        for (auto receiver_id : receiver_ids_) {
          baseaction.push_back(make_action<action::no_operation>(receiver_id));
        }
        break;
      }
    }
  }
  if (ball_log_.size() > 10) {
    ball_log_.pop();
  }
  ball_log_.push({ball.x(), ball.y()});
  return baseaction;
}

bool setplay::finished() {
  return state_ == state::finished;
}
} // agent
} // game
} // ai_server
