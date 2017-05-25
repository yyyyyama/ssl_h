#include <Eigen/Geometry>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/model/command.h"
#include "ai_server/util/math.h"

#include "setplay.h"

namespace ai_server {
namespace game {
namespace agent {

setplay::setplay(const model::world& world, bool is_yellow, unsigned int kicker_id,
                 const std::vector<unsigned int>& receiver_ids)
    : base(world, is_yellow), kicker_id_(kicker_id), receiver_ids_(receiver_ids) {
  kick_ = make_action<action::kick_action>(kicker_id_);
}

std::vector<std::shared_ptr<action::base>> setplay::execute() {
  const auto our_robots   = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  // 使いロボットが全員見えているか確認
  if (our_robots.count(kicker_id_) && std::all_of(receiver_ids_.cbegin(), receiver_ids_.cend(),
                                                  [&our_robots](auto&& receiver_id) {
                                                    return our_robots.count(receiver_id);
                                                  })) {
    using boost::math::constants::pi;
    baseaction_.clear();
    baseaction_.shrink_to_fit();

    const auto& kicker     = our_robots.at(kicker_id_);
    const auto& ball       = world_.ball();
    const double ballysign = ball.y() > 0 ? 1.0 : -1.0;
    double enemygoal       = world_.field().x_max();

    // これより遠いと速度制限
    const double fdir = 600;

    // パス以降の処理でボールの軌道が大きく変わったら的に取られたと判定し、agent終了
    if (static_cast<int>(state_) >= static_cast<int>(state::receive)) {
      auto shooter = our_robots.at(shooter_id_);
      if (std::abs(std::atan2(ball.vy(), ball.vx()) - std::atan2(ballv_.y(), ballv_.x())) >
              1.0 &&
          std::hypot(shooter.x() - ball.x(), shooter.y() - ball.y()) > 3000 &&
          std::hypot(kicker.x() - ball.x(), kicker.y() - ball.y()) > 500) {
        state_ = state::finished;
      }
    }
    ballv_ = {ball.vx(), ball.vy()};

    // キックフラグが立ったら次の状態に移行
    if (kick_->finished()) {
      if (state_ == state::pass) {
        kick_  = make_action<action::kick_action>(shooter_id_);
        state_ = state::receive;
      } else {
        state_ = state::finished;
      }
    }
    // レシーバの指定がない(ダイレクトシュート)時
    if (state_ != state::finished && receiver_ids_.size() == 0) {
      state_      = state::shoot;
      shooter_id_ = kicker_id_;
    }
    switch (state_) {
      case setplay::state::setup: {
        if (positions_.size() == 0) {
          // 初めて呼ばれた時はレシーバたちの位置を決める
          /*現状固定なので動的に求めたい{{{*/
          int i = 0;
          for (auto receiver_id : receiver_ids_) {
            const auto& robot  = our_robots.at(receiver_id);
            const double rad   = -0.4;
            const double theta = ballysign * (i / 2 + 1) * rad * (1.0 - 2 * (i % 2));
            const double dire  = std::atan2(ball.y() - robot.y(), ball.x() - robot.x());
            if (ball.x() > 2000) {
              pos_ = pos::far;
              positions_.push_back(
                  {enemygoal - std::cos(theta) * 2000, std::sin(theta) * 3000, dire});
            } else if (ball.x() < -2000) {
              pos_ = pos::near;
              positions_.push_back({ball.x() + (i + 1) * 1500, std::sin(theta) * 3000, dire});
            } else {
              pos_ = pos::mid;
              positions_.push_back({ball.x() + (i + 1) * 1000, std::sin(theta) * 2000, dire});
            }
            if (positions_[i].x() > enemygoal - 500) {
              positions_[i].x() = enemygoal - 500;
            }
            if (std::hypot(enemygoal - positions_[i].x(), positions_[i].y()) < 1500) {
              positions_[i].x() = enemygoal - 1500;
            }
            i++;
          } /*}}}*/
        }
        int i          = 0;
        bool movedflag = true;
        for (auto receiver_id : receiver_ids_) {
          if (std::hypot(positions_[i].x() - our_robots.at(receiver_id).x(),
                         positions_[i].y() - our_robots.at(receiver_id).y()) > 100) {
            movedflag = false;
            break;
          }
          i++;
        }
        if (std::hypot(ball.x() - kicker.x(), ball.y() - kicker.y()) > 700) {
          movedflag = false;
        }
        if (movedflag) {
          state_ = state::pass;
          double max_dist;
          for (auto receiver_id : receiver_ids_) {
            double min_dist = 100000;
            auto& receiver  = our_robots.at(receiver_id);
            for (auto enemy_robot_p : enemy_robots) {
              auto enemy_robot = std::get<1>(enemy_robot_p);
              if ((ball.x() - enemy_robot.x()) *
                  (receiver.x() - ball.x() - enemy_robot.x() < 0)) {
                Eigen::Vector3d to_shooter(receiver.x() - ball.x(), receiver.y() - ball.y(), 0);
                Eigen::Vector3d to_enemy(enemy_robot.x() - ball.x(), enemy_robot.y() - ball.y(),
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
          // 自ゴール付近で開始するときにパス位置(最終的に計算で求めるようにする)
          if (pos_ == pos::near) {
            passpos_ = {1000, 0};
          } else if (pos_ == pos::mid) {
            auto shooter = our_robots.at(shooter_id_);
            passpos_     = {shooter.x(), shooter.y()};
          } else {
            auto shooter = our_robots.at(shooter_id_);
            passpos_     = {shooter.x(), shooter.y()};
          }
          baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
          for (auto receiver_id : receiver_ids_)
            baseaction_.push_back(make_action<action::no_operation>(receiver_id));
        } else {
          for (i = 0; i < static_cast<double>(receiver_ids_.size()); i++) {
            const auto& receiver = our_robots.at(receiver_ids_[i]);
            auto move            = make_action<action::move>(receiver_ids_[i]);
            const double to_target =
                std::atan2(positions_[i].y() - receiver.y(), positions_[i].x() - receiver.x());
            const int dis =
                std::hypot(positions_[i].x() - receiver.x(), positions_[i].y() - receiver.y());
            bool neflag = dis < 10;

            const int sp = dis < 400 ? dis : fdir;
            move->move_to(receiver.x() + (neflag ? 0 : sp * std::cos(to_target)),
                          receiver.y() + (neflag ? 0 : sp * std::sin(to_target)),
                          std::atan2(ball.y() - receiver.y(), ball.x() - receiver.x()));
            baseaction_.push_back(move);
          }
          if (std::hypot(ball.x() - kicker.x(), ball.y() - kicker.y()) > 500) {
            auto move            = make_action<action::move>(kicker_id_);
            const double to_ball = std::atan2(ball.y() - kicker.y(), ball.x() - kicker.x());
            move->move_to(kicker.x() + 500 * std::cos(to_ball),
                          kicker.y() + 500 * std::sin(to_ball), to_ball);
            baseaction_.push_back(move);
          } else {
            baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
          }
          break;
        }
      }
      case state::pass: {
        kick_->set_mode(action::kick_action::mode::ball);

        bool line_flag = true;

        std::vector<Eigen::Vector2d> targets;
        targets.push_back(passpos_);
        const auto ch = chooselocation(targets, enemy_robots, 500);
        if (ch.x() == 6000 && ch.y() == 6000) {
          line_flag = false;
        }

        const double power = line_flag ? 20 : 150;
        kick_->kick_to(passpos_.x(), passpos_.y());
        kick_->set_dribble(3);
        kick_->set_anglemargin(0.04);
        auto kick_type =
            line_flag ? model::command::kick_type_t::line : model::command::kick_type_t::chip;
        kick_->set_kick_type({kick_type, power});
        baseaction_.push_back(kick_);

        for (auto receiver_id : receiver_ids_) {
          const auto receiver = our_robots.at(receiver_id);
          if (receiver_id != shooter_id_) {
            auto move = make_action<action::move>(receiver_id);
            move->move_to(receiver.x(), receiver.y(),
                          std::atan2(ball.y() - receiver.y(), ball.x() - receiver.x()));
            baseaction_.push_back(move);
          } else {
            const Eigen::Vector3d target = {
                passpos_.x(), passpos_.y(),
                std::atan2(ball.y() - receiver.y(), ball.x() - receiver.x())};
            auto move = make_action<action::move>(shooter_id_);
            const double to_target =
                std::atan2(target.y() - receiver.y(), target.x() - receiver.x());
            const int dis = std::hypot(target.x() - receiver.x(), target.y() - receiver.y());
            bool neflag   = dis < 50;
            const int sp  = dis < 400 ? dis : fdir;
            move->move_to(passpos_.x() + (neflag ? 0 : sp * std::cos(to_target)),
                          passpos_.y() + (neflag ? 0 : sp * std::sin(to_target)),
                          std::atan2(ball.y() - receiver.y(), ball.x() - receiver.x()));
            baseaction_.push_back(move);
          }
        }
        break;
      }
      case state::receive: {
        baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
        for (auto receiver_id : receiver_ids_) {
          if (receiver_id == shooter_id_) {
            const auto& receiver = our_robots.at(receiver_id);
            const double to_rec  = std::hypot(receiver.x() - ball.x(), receiver.y() - ball.y());
            if (to_rec > 200) {
              const Eigen::Vector3d target = seekcrosspoint(
                  {ball.vx(), ball.vy()}, {receiver.x(), receiver.y()}, {ball.x(), ball.y()});
              auto move = make_action<action::move>(shooter_id_);
              const double to_target =
                  std::atan2(target.y() - receiver.y(), target.x() - receiver.x());
              const int dis = std::hypot(target.x() - receiver.x(), target.y() - receiver.y());
              bool neflag   = dis < 50;

              const int sp = dis < 400 ? dis : fdir;
              move->move_to(receiver.x() + (neflag ? 0 : sp * std::cos(to_target)),
                            receiver.y() + (neflag ? 0 : sp * std::sin(to_target)),
                            std::atan2(ball.y() - receiver.y(), ball.x() - receiver.x()));
              move->set_dribble(7);
              baseaction_.push_back(move);
            } else {
              state_ = state::shoot;
            }
          } else {
            baseaction_.push_back(make_action<action::no_operation>(receiver_id));
          }
        }
        if (state_ != state::shoot) {
          break;
        }
      }
      case state::shoot: {
        if (receiver_ids_.size() == 0) {
          std::vector<Eigen::Vector2d> goalpoint;
          goalpoint.push_back({enemygoal, -270});
          goalpoint.push_back({enemygoal, 0});
          goalpoint.push_back({enemygoal, 270});
          Eigen::Vector2d target = chooselocation(goalpoint, enemy_robots);
          const double power     = 50;
          kick_->kick_to(target.x(), target.y());
          kick_->set_dribble(3);
          kick_->set_mode(action::kick_action::mode::ball);
          kick_->set_anglemargin(0.04);
          kick_->set_kick_type({model::command::kick_type_t::line, power});
          baseaction_.push_back(kick_);
        } else {
          baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
        }
        for (auto receiver_id : receiver_ids_) {
          if (receiver_id == shooter_id_) {
            std::vector<Eigen::Vector2d> goalpoint;
            goalpoint.push_back({enemygoal, -270});
            goalpoint.push_back({enemygoal, 0});
            goalpoint.push_back({enemygoal, 270});
            Eigen::Vector2d target = chooselocation(goalpoint, enemy_robots);
            const double power     = 50;
            kick_->kick_to(target.x(), target.y());
            kick_->set_dribble(7);
            kick_->set_mode(action::kick_action::mode::ball);
            kick_->set_anglemargin(0.03);
            kick_->set_kick_type({model::command::kick_type_t::line, power});
            baseaction_.push_back(kick_);
          } else {
            baseaction_.push_back(make_action<action::no_operation>(receiver_id));
          }
        }
        break;
      }
      case state::finished: {
        baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
        for (auto receiver_id : receiver_ids_) {
          baseaction_.push_back(make_action<action::no_operation>(receiver_id));
        }
        break;
      }
    }
  }
  return baseaction_;
}
bool setplay::finished() {
  return state_ == state::finished;
}
// いい感じの場所を選ぶ関数/*{{{*/
Eigen::Vector2d setplay::chooselocation(std::vector<Eigen::Vector2d> targets,
                                        model::world::robots_list enemy_robots, int dist) {
  const auto& ball = world_.ball();
  Eigen::Vector2d decide;
  double max_dist = 0;
  for (auto& target : targets) {
    double min_dist = 100000;
    for (auto& enemy_robot_p : enemy_robots) {
      auto& enemy_robot = std::get<1>(enemy_robot_p);
      // ボールと目的位置の間に敵ロボットがいるかしらべたかったがこれだとがばがばなので要修正
      if ((ball.x() - enemy_robot.x()) * (target.x() - enemy_robot.x()) < 0) {
        const Eigen::Vector3d to_target(target.x() - ball.x(), target.y() - ball.y(), 0);
        const Eigen::Vector3d to_enemy(enemy_robot.x() - ball.x(), enemy_robot.y() - ball.y(),
                                       0);
        const double distance = std::abs(to_target.cross(to_enemy).norm() / to_target.norm());
        if (distance < dist) {
          return {6000, 6000};
        }
        if (distance < min_dist) {
          min_dist = distance;
        }
      }
    }
    if (min_dist > max_dist) {
      max_dist = min_dist;
      decide   = target;
    }
  }
  return decide;
} /*}}}*/
// 直線と点の交点を求める関数/*{{{*/
// to_target:ボールの速度ベクトル
// receiver:レシーバーの位置
// start:ボールの位置
Eigen::Vector3d setplay::seekcrosspoint(Eigen::Vector2d to_target, Eigen::Vector2d receiver,
                                        Eigen::Vector2d start) {
  const Eigen::Vector2d to_res(receiver - start);
  const Eigen::Vector2d normaliz = to_target.normalized();
  const double dot               = normaliz.dot(to_res);
  Eigen::Vector2d tar(start + dot * normaliz);
  const double theta = std::atan2(-normaliz.y(), -normaliz.x());
  return {tar.x(), tar.y(), theta};
} /*}}}*/
} // agent
} // game
} // ai_server
