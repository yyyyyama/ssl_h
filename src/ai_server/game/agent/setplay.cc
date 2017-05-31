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

  // kicker_idが見えない
  if (!our_robots.count(kicker_id_)
      // またはreceiver_idsのロボットが見えない時
      || std::any_of(
             receiver_ids_.cbegin(), receiver_ids_.cend(),
             [&our_robots](auto&& receiver_id) { return !our_robots.count(receiver_id); })) {
    return baseaction_;
  }
  using boost::math::constants::pi;
  baseaction_.clear();

  const Eigen::Vector2d kicker_pos = util::math::position(our_robots.at(kicker_id_));
  const auto& ball                 = world_.ball();
  const Eigen::Vector2d ball_pos   = util::math::position(ball);
  const Eigen::Vector2d ball_vec   = util::math::velocity(ball);
  const double ballysign           = ball.y() > 0 ? 1.0 : -1.0;
  double enemygoal_x               = world_.field().x_max();

  // これより遠いと速度制限
  const double fdir = 600;

  // パス以降の処理でボールの軌道が大きく変わったら的に取られたと判定し、agent終了
  if (state_ >= state::receive) {
    Eigen::Vector2d shooter = util::math::position(our_robots.at(shooter_id_));
    if (std::abs(util::wrap_to_pi(vectorangle(ball_vec) - vectorangle(prev_ball_vel_))) > 1.0 &&
        (shooter - ball_pos).norm() > 3000 && (kicker_pos - ball_pos).norm() > 500) {
      state_ = state::finished;
    }
  }
  prev_ball_vel_ = ball_vec;

  // キックフラグが立ったら次の状態に移行
  if (kick_->finished()) {
    if (state_ == state::pass) {
      receive_ = make_action<action::receive>(shooter_id_);
      kick_    = make_action<action::kick_action>(shooter_id_);
      state_   = state::receive;
    } else {
      state_ = state::finished;
    }
  }
  // レシーバの指定がない(ダイレクトシュート)時
  if (state_ != state::finished && receiver_ids_.empty()) {
    state_      = state::shoot;
    shooter_id_ = kicker_id_;
  }
  switch (state_) {
    case setplay::state::setup: {
      // ロボットを指定位置へ移動させる
      if (positions_.size() == 0) {
        // 初めて呼ばれた時はレシーバたちの位置を決める
        /*現状固定なので動的に求めたい{{{*/
        int i = 0;
        for (auto receiver_id : receiver_ids_) {
          const double rad   = std::abs(ball_pos.y()) > 1500 ? -0.4 : -1.0;
          const double theta = ballysign * (i / 2 + 1) * rad * (1.0 - 2 * (i % 2));
          if (ball_pos.x() > 2000) {
            pos_ = pos::far;
            positions_.push_back(
                {enemygoal_x - std::cos(theta) * 2000, std::sin(theta) * 3000});
          } else if (ball_pos.x() < -2000) {
            pos_ = pos::near;
            positions_.push_back({ball_pos.x() + (i + 2) * 1500, std::sin(theta) * 3000});
          } else {
            pos_ = pos::mid;
            positions_.push_back({ball_pos.x() + (i + 1) * 1000, std::sin(theta) * 2000});
          }
          if (positions_[i].x() > enemygoal_x - 500) {
            positions_[i].x() = enemygoal_x - 500;
          }
          if (std::hypot(enemygoal_x - positions_[i].x(), positions_[i].y()) < 1500) {
            positions_[i].x() = enemygoal_x - 1500;
          }
          i++;
        } /*}}}*/
      }
      int i          = 0;
      bool movedflag = true;
      // ロボットたちが指定位置に移動したか
      for (auto receiver_id : receiver_ids_) {
        if ((positions_[i] - util::math::position(our_robots.at(receiver_id))).norm() > 100) {
          movedflag = false;
          break;
        }
        i++;
      }
      if ((ball_pos - kicker_pos).norm() > 700) {
        movedflag = false;
      }

      // ロボットが指定位置に移動していたらパス状態へ移行
      if (movedflag) {
        state_ = state::pass;
        double max_dist;
        for (auto receiver_id : receiver_ids_) {
          double min_dist = std::numeric_limits<double>::max();
          auto& receiver  = our_robots.at(receiver_id);
          for (auto enemy_robot_p : enemy_robots) {
            auto enemy_robot = std::get<1>(enemy_robot_p);
            if ((ball_pos.x() - enemy_robot.x()) *
                    (receiver.x() - ball_pos.x() - enemy_robot.x()) <
                0) {
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
        // ロボットが移動できていなかったら移動を続ける
        for (i = 0; i < receiver_ids_.size(); i++) {
          const Eigen::Vector2d receiver =
              util::math::position(our_robots.at(receiver_ids_[i]));
          auto move              = make_action<action::move>(receiver_ids_[i]);
          const double to_target = vectorangle(positions_[i] - receiver);
          const int dist         = (positions_[i] - receiver).norm();
          bool close_flag        = dist < 10;

          const int speed = dist < 400 ? dist : fdir;
          move->move_to(receiver.x() + (close_flag ? 0 : speed * std::cos(to_target)),
                        receiver.y() + (close_flag ? 0 : speed * std::sin(to_target)),
                        vectorangle(ball_pos - receiver));
          baseaction_.push_back(move);
        }
        if ((ball_pos - kicker_pos).norm() > 500) {
          auto move            = make_action<action::move>(kicker_id_);
          const double to_ball = vectorangle(ball_pos - kicker_pos);
          move->move_to(kicker_pos.x() + 500 * std::cos(to_ball),
                        kicker_pos.y() + 500 * std::sin(to_ball), to_ball);
          baseaction_.push_back(move);
        } else {
          baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
        }
        break;
      }
    }
    case state::pass: {
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

      const double power = line_flag ? 50 : 150;
      kick_->kick_to(passpos_.x(), passpos_.y());
      kick_->set_dribble(0);
      kick_->set_anglemargin(0.04);
      auto kick_type =
          line_flag ? model::command::kick_type_t::line : model::command::kick_type_t::chip;
      kick_->set_kick_type({kick_type, power});
      baseaction_.push_back(kick_);

      for (auto receiver_id : receiver_ids_) {
        const auto receiver = util::math::position(our_robots.at(receiver_id));
        if (receiver_id != shooter_id_) {
          auto move = make_action<action::move>(receiver_id);
          move->move_to(receiver.x(), receiver.y(), vectorangle(ball_pos - receiver));
          baseaction_.push_back(move);
        } else {
          const Eigen::Vector2d target = {passpos_.x(), passpos_.y()};
          auto move                    = make_action<action::move>(shooter_id_);
          const double to_target       = vectorangle(target - receiver);
          const int dist               = (target - receiver).norm();
          bool close_flag              = dist < 50;
          const int speed              = dist < 400 ? dist : fdir;
          move->move_to(passpos_.x() + (close_flag ? 0 : speed * std::cos(to_target)),
                        passpos_.y() + (close_flag ? 0 : speed * std::sin(to_target)),
                        vectorangle(ball_pos - receiver));
          baseaction_.push_back(move);
        }
      }
      break;
    }
    case state::receive: {
      // パスを受け取る
      baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
      for (auto receiver_id : receiver_ids_) {
        if (receiver_id == shooter_id_) {
          if (!receive_->finished()) {
            receive_->set_dribble(9);
            baseaction_.push_back(receive_);
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
      // シュートを撃つ
      if (receiver_ids_.size() == 0) {
        Eigen::Vector2d target = {enemygoal_x, -270 * ballysign};
        const double power     = 50;
        kick_->kick_to(target.x(), target.y());
        kick_->set_dribble(0);
        kick_->set_mode(action::kick_action::mode::ball);
        kick_->set_anglemargin(0.04);
        kick_->set_kick_type({model::command::kick_type_t::line, power});
        baseaction_.push_back(kick_);
      } else {
        baseaction_.push_back(make_action<action::no_operation>(kicker_id_));
      }
      for (auto receiver_id : receiver_ids_) {
        if (receiver_id == shooter_id_) {
          Eigen::Vector2d target = {enemygoal_x, -270 * ballysign};
          const double power     = 50;
          kick_->kick_to(target.x(), target.y());
          kick_->set_dribble(0);
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
      // 終了
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
// いい感じの場所を選ぶ関数/*{{{*/
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
} /*}}}*/
// ベクトルを渡すと角度を返してくれるもの/*{{{*/
double setplay::vectorangle(Eigen::Vector2d vec) {
  return std::atan2(vec.y(), vec.x());
} /*}}}*/
} // agent
} // game
} // ai_server
