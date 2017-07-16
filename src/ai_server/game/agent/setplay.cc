#include <Eigen/Geometry>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <limits>

#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/model/command.h"
#include "ai_server/util/math.h"
#include "ai_server/util/math/to_vector.h"

#include "setplay.h"
#include <iostream>

namespace ai_server {
namespace game {
namespace agent {

setplay::setplay(const model::world& world, bool is_yellow, unsigned int kicker_id,
                 const std::vector<unsigned int>& receiver_ids)
    : base(world, is_yellow), kicker_id_(kicker_id), receiver_ids_(receiver_ids) {
  kick_       = make_action<action::kick_action>(kicker_id_);
  shooter_id_ = kicker_id_;
  receive_    = make_action<action::receive>(kicker_id_);
  shoot_pos   = {world_.field().x_max(), 0};
}

std::vector<std::shared_ptr<action::base>> setplay::execute() {
  const auto our_robots   = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();

  baseaction_.clear();

  // kicker_idが見えない
  if (!our_robots.count(kicker_id_)
      // またはreceiver_idsのロボットが見えない時
      || std::any_of(
             receiver_ids_.cbegin(), receiver_ids_.cend(),
             [&our_robots](auto&& receiver_id) { return !our_robots.count(receiver_id); })) {
    return baseaction_;
  }
  using boost::math::constants::pi;

  const Eigen::Vector2d kicker_pos = util::math::position(our_robots.at(kicker_id_));
  const auto& kicker               = our_robots.at(kicker_id_);
  const auto& ball                 = world_.ball();
  const Eigen::Vector2d ball_pos   = util::math::position(ball);
  const Eigen::Vector2d ball_vel   = util::math::velocity(ball);
  const double ballysign           = (ball.y() > 0 || std::abs(ball.y()) < 250) ? 1.0 : -1.0;
  double enemygoal_x               = world_.field().x_max();

  // パス以降の処理でボールの軌道が大きく変わったら的に取られたと判定し、agent終了
  if (state_ >= state::receive) {
    Eigen::Vector2d shooter_pos = util::math::position(our_robots.at(shooter_id_));
    // if (std::abs(util::wrap_to_pi(vectorangle(ball_vel) - vectorangle(prev_ball_vel_))) > 0.5
    // &&
    //     (kicker_pos - ball_pos).norm() > 500 && ball_vel.norm() > 1500 &&
    //     (std::abs(ball_pos.x()) > 500 || std::abs(ball_pos.y()) > 500) &&
    //     !receive_->finished()) {

    if (!receiver_ids_.empty() &&
        std::abs(util::wrap_to_pi(vectorangle(ball_pos - kicker_pos) -
                                  vectorangle(shooter_pos - kicker_pos))) > 0.8) {
      std::cerr << "fi1" << std::abs(util::wrap_to_pi(vectorangle(ball_pos - kicker_pos) -
                                                      vectorangle(shooter_pos - kicker_pos)))
                << ", " << (kicker_pos - ball_pos).norm() << ", " << ball_vel.norm() << ", "
                << std::abs(ball_pos.x()) << ", " << std::abs(ball_pos.y()) << std::endl;
      std::cerr << "finished1" << std::endl;
      state_ = state::finished;
    }
    if ((shooter_pos - ball_pos).norm() < 500)
      neflag = true;
    else if (neflag && (shooter_pos - ball_pos).norm() > 1000) {
      state_ = state::finished;
      std::cerr << "finished2" << std::endl;
    } else if (!receiver_ids_.empty() && state_ >= state::receive &&
               (ball_pos - kicker_pos).norm() > 500 && (ball_pos - shooter_pos).norm() > 200 &&
               ball_vel.norm() < 200 && !receive_->finished()) {
      std::cerr << "fi3" << (ball_pos - shooter_pos).norm() << ", " << ball_vel.norm()
                << std::endl;
      state_ = state::finished;
      std::cerr << "finished3" << std::endl;
    }
  }
  prev_ball_vel_ = ball_vel;

  // キックフラグが立ったら次の状態に移行
  if (kick_->finished()) {
    std::cout << "state" << static_cast<int>(state_) << std::endl;
    if (state_ == state::pass) {
      std::cout << "kickaa" << std::endl;
      kick_  = make_action<action::kick_action>(shooter_id_);
      state_ = state::receive;
    } else if (state_ == state::shoot) {
      state_ = state::finished;
      std::cout << "3torareta" << std::endl;
    }
  }
  std::cout << "start_setplay4" << std::endl;
  // レシーバの指定がない(ダイレクトシュート)時
  if (state_ != state::finished && receiver_ids_.empty()) {
    state_      = state::shoot;
    shooter_id_ = kicker_id_;
  }
  std::cout << "switch" << std::endl;
  switch (state_) {
    case setplay::state::setup: {
      // ロボットを指定位置へ移動させる
      if (positions_.size() == 0) {
        // 初めて呼ばれた時はレシーバたちの位置を決める
        // 現状固定なので動的に求めたい{{{
        int i = 0;
        for (auto receiver_id : receiver_ids_) {
          const double rad   = std::abs(ball_pos.y()) > 1500 ? -0.4 : -0.7;
          const double theta = ballysign * (i / 2 + 1) * rad * (1.0 - 2 * (i % 2));
          std::cout << "myonmyon" << receiver_id << ", " << i << ", " << theta << std::endl;
          const double kick_off_coe = ball_pos.norm() > 250 ? 1.0 : -1.0;
          if (ball_pos.x() > 2000) {
            pos_ = pos::far;
            positions_.push_back(
                {enemygoal_x - std::cos(theta) * (i + 1) * 3000, std::sin(theta) * 2000});
            // {enemygoal_x - std::cos(theta) * 2000, std::sin(theta) * 2000});
          } else if (ball_pos.x() < -2000) {
            pos_ = pos::near;
            // positions_.push_back({ball_pos.x() + (i + 2) * 1500, std::sin(theta) * 2000});
            positions_.push_back({ball_pos.x() + (i + 2) * 1500, std::sin(theta) * 3000});
          } else {
            pos_ = pos::mid;
            // positions_.push_back({ball_pos.x() + (i + 1) * 1000, std::sin(theta) * 2000});
            positions_.push_back(
                {ball_pos.x() + (i + 1) * 1000 * kick_off_coe, std::sin(theta) * 3000});
          }
          if (positions_[i].x() > enemygoal_x - 500) {
            positions_[i].x() = enemygoal_x - 500;
          }
          if (std::hypot(enemygoal_x - positions_[i].x(), positions_[i].y()) < 1500) {
            positions_[i].x() = enemygoal_x - 1500;
          }
          i++;
        } // }}}
      }
      state_      = state::pass;
      shooter_id_ = receiver_ids_[shooter_num_];
      passpos_    = {positions_[shooter_num_].x(), positions_[shooter_num_].y()};
      receive_    = make_action<action::receive>(shooter_id_);
    }
    case state::pass: {
      std::cout << "nstate_pass" << std::endl;
      // パスを出す
      kick_->set_mode(action::kick_action::mode::ball);

      bool line_flag = true;

      std::vector<Eigen::Vector2d> targets;
      targets.push_back(passpos_);
      // 進路上の近くに敵ロボットがいたらチップキック
      const auto ch = find_location(targets, enemy_robots, 500);
      if (ch.x() == std::numeric_limits<double>::max()) {
        line_flag = false;
      }

      bool movedflag = true;
      // ロボットたちが指定位置に移動したか
      std::cout << "nnn"
                << (positions_[0] - util::math::position(our_robots.at(shooter_id_))).norm()
                << std::endl;
      if ((positions_[shooter_num_] - util::math::position(our_robots.at(shooter_id_))).norm() > 500 &&
          receive_flag_) {
        movedflag = false;
      }
      //
      if (kick_->state() == action::kick_action::running_state::kick && !movedflag) {
        baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
      } else {
        receive_flag_ = true;
        // const double power = line_flag ? 45 : 60;
        // const double power = line_flag ? 50 : 120;
        const double power = line_flag ? 7 : 120;
        kick_->kick_to(passpos_.x(), passpos_.y());
        kick_->set_dribble(3);
        kick_->set_angle_margin(0.10);
        kick_->set_stop_ball(true);
        auto kick_type =
            line_flag ? model::command::kick_type_t::line : model::command::kick_type_t::chip;
        kick_->set_kick_type({kick_type, power});
        baseaction_.push_back(kick_);
      }

      int i = 0;
      for (auto receiver_id : receiver_ids_) {
        const auto receiver_pos = util::math::position(our_robots.at(receiver_id));
        if (receiver_id != shooter_id_) {
          auto move = make_action<action::move>(receiver_id);
          move->move_to(positions_[i].x(), positions_[i].y(),
                        vectorangle(ball_pos - receiver_pos));
          baseaction_.push_back(move);
        } else {
          if (kick_->state() == action::kick_action::running_state::kick && movedflag) {
            std::cout << "iaia" << std::endl;
            receive_->set_dribble(9);
            receive_->set_passer(kicker_id_);
            if (pos_ == pos::far) {
              receive_->set_shoot({enemygoal_x, 0});
              const double power = 255;
              receive_->set_kick_type({model::command::kick_type_t::line, power});
            }
            baseaction_.push_back(receive_);
          } else {
            std::cout << "passposx" << passpos_.x() << "passposy" << passpos_.y() << std::endl;
            const Eigen::Vector2d target = {passpos_.x(), passpos_.y()};
            auto move                    = make_action<action::move>(shooter_id_);
            const double to_target       = vectorangle(target - receiver_pos);
            const int dist               = (target - receiver_pos).norm();
            bool close_flag              = dist < 50;
            move->move_to(passpos_.x() + (close_flag ? 0 : dist * std::cos(to_target)),
                          passpos_.y() + (close_flag ? 0 : dist * std::sin(to_target)),
                          vectorangle(ball_pos - receiver_pos));
            baseaction_.push_back(move);
          }
        }
        i++;
      }
      break;
    }
    case state::receive: {
      // パスを受け取る
      std::cout << "state_receive" << std::endl;
      baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
      for (auto receiver_id : receiver_ids_) {
        if (receiver_id == shooter_id_) {
          if (!receive_->finished()) {
            std::cout << "shsh1" << ball_pos.y() << std::endl;
            receive_->set_passer(kicker_id_);
            receive_->set_dribble(9);
            if (pos_ == pos::far) {
              receive_->set_shoot({enemygoal_x, 0});
              const double power = 255;
              receive_->set_kick_type({model::command::kick_type_t::line, power});
            }
            baseaction_.push_back(receive_);
          } else {
            std::cout << "shsh2" << std::endl;
            // キーパー探索
            const Eigen::Vector2d shooter_pos =
                util::math::position(our_robots.at(shooter_id_));
            double min_dist = std::numeric_limits<double>::max();
            double to_keeper_theta;
            for (auto& enemy_robot_p : enemy_robots) {
              auto& enemy_robot = std::get<1>(enemy_robot_p);
              // const double distance = (enemy_robot - Eigen::Vector2d{enemygoal_x, 0}).norm();
              const double distance =
                  std::hypot(enemy_robot.x() - enemygoal_x, enemy_robot.y() - 0);
              if (distance > 1000) {
                continue;
              }
              if (distance < min_dist) {
                min_dist        = distance;
                to_keeper_theta = std::atan2(enemy_robot.y() - shooter_pos.y(),
                                             enemy_robot.x() - shooter_pos.x());
              }
            }
            std::cout << "shpos hoge1 " << to_keeper_theta << std::endl;
            if (min_dist != std::numeric_limits<double>::max()) {
              std::cout << "shpos hoge2" << std::endl;
              const double goal_max_plus_theta =
                  vectorangle(Eigen::Vector2d{enemygoal_x, 500} - ball_pos);
              const double goal_max_minus_theta =
                  vectorangle(Eigen::Vector2d{enemygoal_x, -500} - ball_pos);
              if (to_keeper_theta < goal_max_plus_theta &&
                  to_keeper_theta > goal_max_minus_theta) {
                std::cout << "shpos hoge3" << std::endl;
                double shoot_theta;
                if (goal_max_plus_theta - to_keeper_theta >
                    to_keeper_theta - goal_max_minus_theta) {
                  shoot_theta = (to_keeper_theta + goal_max_plus_theta) / 2;
                  std::cout << "shpos hoge4 " << shoot_theta << std::endl;
                } else {
                  shoot_theta = (to_keeper_theta + goal_max_minus_theta) / 2;
                  std::cout << "shpos " << to_keeper_theta << ", " << goal_max_plus_theta
                            << ", " << goal_max_minus_theta << std::endl;
                  std::cout << "shpos hoge5 " << shoot_theta << std::endl;
                }
                const Eigen::Vector2d shoot_vec = {std::cos(shoot_theta),
                                                   std::sin(shoot_theta)};
                std::cout << "shpos hoge6 " << shoot_vec.x() << ", " << shoot_vec.y()
                          << std::endl;
                shoot_pos = {enemygoal_x,
                             ball_pos.y() +
                                 shoot_vec.y() * (enemygoal_x - ball_pos.x()) / shoot_vec.x()};
                std::cout << "shpos hoge8 " << enemygoal_x - ball_pos.x() << std::endl;
                std::cout << "shpos hoge7 " << shoot_pos.x() << ", " << shoot_pos.y()
                          << std::endl;
              }
            }
            // shoot_pos = {enemygoal_x, 0};
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
      std::cout << "state_shoot" << shoot_pos.y() << std::endl;
      // シュートを撃つ
      std::cout << "shoot_pos" << shoot_pos.x() << ", " << shoot_pos.y() << std::endl;
      if (receiver_ids_.size() == 0) {
        // Eigen::Vector2d target = {enemygoal_x, -400 * ballysign};
        Eigen::Vector2d target = {enemygoal_x, 0 * ballysign};
        const double power     = 255;
        // kick_->kick_to(target.x(), target.y());
        kick_->kick_to(shoot_pos.x(), shoot_pos.y());
        kick_->set_dribble(0);
        kick_->set_mode(action::kick_action::mode::ball);
        kick_->set_angle_margin(0.15);
        kick_->set_kick_type({model::command::kick_type_t::line, power});
        baseaction_.push_back(kick_);
      } else {
        baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
      }
      for (auto receiver_id : receiver_ids_) {
        if (receiver_id == shooter_id_) {
          Eigen::Vector2d target = {enemygoal_x, 0 * ballysign};
          const double power     = 255;
          // kick_->kick_to(target.x(), target.y());
          kick_->kick_to(shoot_pos.x(), shoot_pos.y());
          kick_->set_dribble(7);
          kick_->set_mode(action::kick_action::mode::ball);
          kick_->set_angle_margin(0.15);
          kick_->set_kick_type({model::command::kick_type_t::line, power});
          baseaction_.push_back(kick_);
        } else {
          baseaction_.push_back(make_action<action::no_operation>(receiver_id));
        }
      }
      break;
    }
    case state::finished: {
      // 終了
      std::cerr << "finish" << std::endl;
      baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
      for (auto receiver_id : receiver_ids_) {
        baseaction_.push_back(make_action<action::no_operation>(receiver_id));
      }
      break;
    }
  }
  return baseaction_;
}
bool setplay::finished() {
  return state_ == state::finished;
}
// いい感じの場所を選ぶ関数{{{
Eigen::Vector2d setplay::find_location(std::vector<Eigen::Vector2d> targets,
                                       model::world::robots_list enemy_robots, int dist) {
  const auto& ball = world_.ball();
  Eigen::Vector2d result;
  double max_dist = 0;
  for (auto& target : targets) {
    double min_dist = std::numeric_limits<double>::max();
    for (auto& enemy_robot_p : enemy_robots) {
      auto& enemy_robot = std::get<1>(enemy_robot_p);
      // ボールと目的位置の間に敵ロボットがいるかしらべたかったがこれだとがばがばなので要修正
      if ((ball.x() - enemy_robot.x()) * (target.x() - enemy_robot.x()) < 0 &&
          (ball.y() - enemy_robot.y()) * (target.y() - enemy_robot.y()) < 0) {
        const Eigen::Vector3d to_target(target.x() - ball.x(), target.y() - ball.y(), 0);
        const Eigen::Vector3d to_enemy(enemy_robot.x() - ball.x(), enemy_robot.y() - ball.y(),
                                       0);
        const double distance = std::abs(to_target.cross(to_enemy).norm() / to_target.norm());
        if (distance < dist) {
          return {std::numeric_limits<double>::max(), 0};
        }
        if (distance < min_dist) {
          min_dist = distance;
        }
      }
    }
    if (min_dist > max_dist) {
      max_dist = min_dist;
      result   = target;
    }
  }
  return result;
} // }}}
// ベクトルを渡すと角度を返してくれるもの{{{
double setplay::vectorangle(Eigen::Vector2d vec) {
  return std::atan2(vec.y(), vec.x());
} // }}}
} // agent
} // game
} // ai_server
