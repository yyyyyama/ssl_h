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
  kick_                            = make_action<action::kick_action>(kicker_id_);
  shooter_id_                      = 0;
  shooter_num_                     = 0;
  receive_                         = make_action<action::receive>(kicker_id_);
  shoot_pos                        = {world_.field().x_max(), 0};
  const auto our_robots            = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const Eigen::Vector2d kicker_pos = util::math::position(our_robots.at(kicker_id_));

  const auto& ball = world_.ball();
  ballysign        = (ball.y() > 0 || std::abs(ball.y()) < 250) ? 1.0 : -1.0;
  // 乱数代わり
  mode_ = static_cast<int>(kicker_pos.x()) % 4;
}

std::vector<unsigned int> setplay::free_robots() const {
  return free_robots_;
}
void setplay::set_mode(int mode_num) {
  mode_ = mode_num;
}

void setplay::set_direct(bool is_direct) {
  is_direct_ = is_direct;
  mode_      = 0;
}

std::vector<std::shared_ptr<action::base>> setplay::execute() {
  free_robots_.clear();
  const auto our_robots   = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  std::vector<std::shared_ptr<action::base>> baseaction;

  // kicker_idが見えない
  if (!our_robots.count(kicker_id_)
      // またはreceiver_idsのロボットが見えない時
      ||
      !std::all_of(
          receiver_ids_.cbegin(), receiver_ids_.cend(),
          [&our_robots](auto&& receiver_id) { return our_robots.count(receiver_id); })) {
    return baseaction;
  }
  using boost::math::constants::pi;
  using boost::math::constants::two_pi;

  const Eigen::Vector2d kicker_pos = util::math::position(our_robots.at(kicker_id_));
  const auto& kicker               = our_robots.at(kicker_id_);
  const auto& ball                 = world_.ball();
  const Eigen::Vector2d ball_pos   = util::math::position(ball);
  const Eigen::Vector2d ball_vel   = util::math::velocity(ball);
  double enemygoal_x               = world_.field().x_max();

  // パス以降の処理でボールの軌道が大きく変わったら的に取られたと判定し、agent終了
  if (state_ >= state::receive) {
    Eigen::Vector2d shooter_pos = util::math::position(our_robots.at(shooter_id_));

    if (!receiver_ids_.empty() &&
        std::abs(util::wrap_to_pi(vectorangle(ball_pos - kicker_pos) -
                                  vectorangle(shooter_pos - kicker_pos))) > 0.8) {
      state_ = state::finished;
    }
    if ((shooter_pos - ball_pos).norm() < 500)
      neflag = true;
    else if (neflag && (shooter_pos - ball_pos).norm() > 1000) {
      state_ = state::finished;
    } else if (!receiver_ids_.empty() && state_ >= state::receive &&
               (ball_pos - kicker_pos).norm() > 500 && (ball_pos - shooter_pos).norm() > 200 &&
               ball_vel.norm() < 200 && !receive_->finished()) {
      state_ = state::finished;
    }
  }
  prev_ball_vel_ = ball_vel;

  // キックフラグが立ったら次の状態に移行
  if (kick_->finished()) {
    if (state_ == state::pass) {
      get_ball_ = make_action<action::get_ball>(shooter_id_);
      state_    = state::receive;
    } else if (state_ == state::shoot) {
      state_ = state::finished;
    }
  }

  std::vector<Eigen::Vector2d> targets = {{enemygoal_x, 0}};
  const int ch                         = chose_location(targets, enemy_robots, 200);
  if (ch != std::numeric_limits<int>::max() && is_direct_ &&
      std::abs(vectorangle(shoot_pos - kicker_pos)) < pi<double>() / 4) {
    try_direct = true;
    passpos_   = {enemygoal_x + 250, 0};
  }
  // レシーバの指定がない(ダイレクトシュート)時
  if (state_ != state::finished && receiver_ids_.empty()) {
    state_      = state::shoot;
    shooter_id_ = kicker_id_;
  }
  constexpr int run_dist = 3500;
  switch (state_) {
    case setplay::state::setup: {
      // ロボットを指定位置へ移動させる
      if (positions_.size() == 0) {
        // 初めて呼ばれた時はレシーバたちの位置を決める
        // 現状固定なので動的に求めたい{{{
        int i = 0;
        for (auto receiver_id : receiver_ids_) {
          const double rad          = std::abs(ball_pos.y()) > 1500 ? -0.4 : -0.7;
          const double theta        = ballysign * (i / 2 + 1) * rad * (1.0 - 2 * (i % 2));
          const double kick_off_coe = ball_pos.norm() > 250 ? 1.0 : -1.0;
          const double dy =
              6000 / (receiver_ids_.size() + 1) / (std::abs(ball_pos.y()) > 1500 ? 1 : 2);
          const double dx = 2000;
          // コーナー
          constexpr int far_max[] = {3000, 4000};
          if (ball_pos.x() > 2000) {
            pos_ = pos::far;
            switch (mode_) {
              case (1): {
                const Eigen::Vector2d tmp_pos = {ball_pos.x() - (i + 1) * dx - run_dist,
                                                 -ballysign * (3000 - (i + 1) * dy)};
                double mid_add = 1;
                if (std::abs(ball_pos.y()) < 1500) {
                  mid_add = i % 2 ? -1 : 1;
                }
                positions_.push_back({tmp_pos.x() < ball_pos.x() - far_max[mode_]
                                          ? ball_pos.x() - far_max[mode_]
                                          : tmp_pos.x(),
                                      tmp_pos.y() * mid_add});
                break;
              }
              default: {
                const Eigen::Vector2d tmp_pos = {ball_pos.x() - (i + 1) * dx,
                                                 -ballysign * (3000 - (i + 1) * dy)};
                double mid_add = 1;
                if (std::abs(ball_pos.y()) < 1500) {
                  mid_add = i % 2 ? -1 : 1;
                }
                positions_.push_back({tmp_pos.x() < ball_pos.x() - far_max[mode_]
                                          ? ball_pos.x() - far_max[mode_]
                                          : tmp_pos.x(),
                                      tmp_pos.y() * mid_add});
                break;
              }
            }
          } else if (ball_pos.x() < -3000) {
            pos_                          = pos::near;
            const Eigen::Vector2d tmp_pos = {ball_pos.x() + (i + 2) * 2000,
                                             -ballysign * (3000 - (i + 1) * dy)};
            double mid_add = 1;
            if (std::abs(ball_pos.y()) < 1500) {
              mid_add = i % 2 ? -1 : 1;
            }
            positions_.push_back({tmp_pos.x(), tmp_pos.y() * mid_add});
          } else {
            pos_                          = pos::mid;
            const Eigen::Vector2d tmp_pos = {ball_pos.x() + (i + 1) * 2000 * kick_off_coe,
                                             -ballysign * (3000 - (i + 1) * dy)};
            double mid_add = 1;
            if (std::abs(ball_pos.y()) < 1500) {
              mid_add = i % 2 ? -1 : 1;
            }
            positions_.push_back(
                {tmp_pos.x() > 3000 ? 3000 : tmp_pos.x(), tmp_pos.y() * mid_add});
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
      if ((shoot_pos - ball_pos).norm() > 200 && change_count_ < 5 &&
          !(mode_ == 1 && pos_ == pos::far)) {
        const int chose_num = chose_location(positions_, enemy_robots);
        if (shooter_num_ != chose_num && chose_num < receiver_ids_.size()) {
          if ((chose_num + 1) < receiver_ids_.size() / 3 * 2) {
            kick_        = make_action<action::kick_action>(kicker_id_);
            shooter_num_ = chose_num;
            shooter_id_  = receiver_ids_[shooter_num_];
            passpos_     = {positions_[shooter_num_].x(), positions_[shooter_num_].y()};
            receive_     = make_action<action::receive>(shooter_id_);
            change_count_++;
          }
        }
      }
      // パスを出す
      kick_->set_mode(action::kick_action::mode::ball);

      bool line_flag = true;

      std::vector<Eigen::Vector2d> targets;
      targets.push_back(passpos_);
      // 進路上の近くに敵ロボットがいたらチップキック
      const int ch = chose_location(targets, enemy_robots, 500);

      if (ch == std::numeric_limits<int>::max()) {
        shooter_id_ = receiver_ids_[shooter_num_];
        passpos_    = {positions_[shooter_num_].x(), positions_[shooter_num_].y()};
        receive_    = make_action<action::receive>(shooter_id_);
        line_flag   = false;
      }

      bool movedflag = true;
      // ロボットたちが指定位置に移動したか
      if ((positions_[shooter_num_] - util::math::position(our_robots.at(shooter_id_))).norm() >
              500 &&
          !receive_flag_) {
        movedflag = false;
      }

      if (mode_ == 1 && !receive_flag_) {
        // レシーバ選択
      }

      // 何時でもけれる状態で移動を待機する
      const double power = line_flag ? 60 : 255;
      if (mode_ == 1 && pos_ == pos::far)
        kick_->kick_to(passpos_.x() + run_dist - 1000, passpos_.y());
      else
        kick_->kick_to(passpos_.x(), passpos_.y());
      kick_->set_dribble(3);
      kick_->set_angle_margin(0.10);
      kick_->set_stop_ball(true);
      auto kick_type =
          line_flag ? model::command::kick_type_t::line : model::command::kick_type_t::chip;
      if (kick_->state() >= action::kick_action::running_state::round && !movedflag) {
        kick_type = model::command::kick_type_t::none;
      }
      if (movedflag) {
        receive_flag_ = true;
      }

      { // receiverの垂線
        const Eigen::Vector2d shooter_pos = util::math::position(our_robots.at(shooter_id_));
        Eigen::Vector2d normalize;
        Eigen::Vector2d position;
        if (ball_vel.norm() < 500) { // passerの正面に移動したい
          normalize = Eigen::Vector2d{std::cos(kicker.theta()), std::sin(kicker.theta())};
          position  = kicker_pos;
        } else { //ボールの移動予測地点に移動したい
          normalize = ball_vel.normalized();
          position  = ball_pos;
        }

        //対象とreceiverの距離
        const auto length = shooter_pos - position;
        //内積より,対象と自分の直交する位置
        const auto dot    = normalize.dot(length);
        const auto target = (position + dot * normalize);
        if (mode_ == 1 && pos_ == pos::far && receive_flag_) {
          constexpr int success_lenge = 500;
          if ((target - shooter_pos).norm() < success_lenge) {
            kick_type = line_flag ? model::command::kick_type_t::line
                                  : model::command::kick_type_t::chip;
          } else {
            kick_type = model::command::kick_type_t::none;
          }
        }
      }
      kick_->set_kick_type({kick_type, power});
      baseaction.push_back(kick_);

      int i = 0;
      for (auto receiver_id : receiver_ids_) {
        const auto receiver_pos = util::math::position(our_robots.at(receiver_id));
        if (receiver_id != shooter_id_) {
          auto move = make_action<action::move>(receiver_id);
          move->move_to(positions_[i].x(), positions_[i].y(),
                        vectorangle(ball_pos - receiver_pos));
          baseaction.push_back(move);
        } else {
          if (kick_->state() == action::kick_action::running_state::kick && mode_ == 1 &&
              pos_ == pos::far && receive_flag_) {
            receive_->set_passer(kicker_id_);
            receive_->set_dribble(2);
            receive_->set_shoot({enemygoal_x, ballysign * 400});
            const double power = 255;
            receive_->set_kick_type({model::command::kick_type_t::line, power});
            baseaction.push_back(receive_);
          } else if (try_direct) {
            const Eigen::Vector2d target = {positions_[receiver_id].x(),
                                            positions_[receiver_id].y()};
            auto move              = make_action<action::move>(shooter_id_);
            const double to_target = vectorangle(target - receiver_pos);
            const int dist         = (target - receiver_pos).norm();
            bool close_flag        = dist < 50;
            move->move_to(
                positions_[receiver_id].x() + (close_flag ? 0 : dist * std::cos(to_target)),
                positions_[receiver_id].y() + (close_flag ? 0 : dist * std::sin(to_target)),
                vectorangle(ball_pos - receiver_pos));
            baseaction.push_back(move);

          } else if (kick_->state() == action::kick_action::running_state::kick && movedflag &&
                     mode_ == 0) {
            receive_->set_dribble(9);
            receive_->set_passer(kicker_id_);
            if (pos_ == pos::far) {
              receive_->set_dribble(2);
              receive_->set_shoot({enemygoal_x, ballysign * 400});
              const double power = 255;
              receive_->set_kick_type({model::command::kick_type_t::line, power});
            }
            baseaction.push_back(receive_);
          } else {
            const Eigen::Vector2d target = {passpos_.x(), passpos_.y()};
            auto move                    = make_action<action::move>(shooter_id_);
            const double to_target       = vectorangle(target - receiver_pos);
            const int dist               = (target - receiver_pos).norm();
            bool close_flag              = dist < 50;
            move->move_to(passpos_.x() + (close_flag ? 0 : dist * std::cos(to_target)),
                          passpos_.y() + (close_flag ? 0 : dist * std::sin(to_target)),
                          vectorangle(ball_pos - receiver_pos));
            baseaction.push_back(move);
          }
        }
        i++;
      }
      break;
    }
    case state::receive: {
      // パスを受け取る
      free_robots_.push_back(kicker_id_);
      for (auto receiver_id : receiver_ids_) {
        if (receiver_id == shooter_id_ && !try_direct) {
          if (!receive_->finished()) {
            receive_->set_passer(kicker_id_);
            receive_->set_dribble(9);
            if (pos_ == pos::far) {
              receive_->set_dribble(2);
              receive_->set_shoot({enemygoal_x, ballysign * 400});
              const double power = 255;
              receive_->set_kick_type({model::command::kick_type_t::line, power});
            }
            baseaction.push_back(receive_);
          } else {
            const Eigen::Vector2d shooter_pos =
                util::math::position(our_robots.at(shooter_id_));
            double min_dist = std::numeric_limits<double>::max();
            double to_keeper_theta;
            for (auto& enemy_robot_p : enemy_robots) {
              auto& enemy_robot = std::get<1>(enemy_robot_p);
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
            if (min_dist != std::numeric_limits<double>::max()) {
              const double goal_max_plus_theta =
                  vectorangle(Eigen::Vector2d{enemygoal_x, 500} - ball_pos);
              const double goal_max_minus_theta =
                  vectorangle(Eigen::Vector2d{enemygoal_x, -500} - ball_pos);
              if (to_keeper_theta < goal_max_plus_theta &&
                  to_keeper_theta > goal_max_minus_theta) {
                double shoot_theta;
                if (goal_max_plus_theta - to_keeper_theta >
                    to_keeper_theta - goal_max_minus_theta) {
                  shoot_theta = (to_keeper_theta + goal_max_plus_theta) / 2;
                } else {
                  shoot_theta = (to_keeper_theta + goal_max_minus_theta) / 2;
                }
                const Eigen::Vector2d shoot_vec = {std::cos(shoot_theta),
                                                   std::sin(shoot_theta)};
                shoot_pos = {enemygoal_x,
                             ball_pos.y() +
                                 shoot_vec.y() * (enemygoal_x - ball_pos.x()) / shoot_vec.x()};
              }
            }
            state_ = state::shoot;
          }
        } else {
          if (free_robots_.size() < 3) free_robots_.push_back(receiver_id);
        }
      }
      if (state_ != state::shoot) {
        break;
      }
    }
    case state::shoot: {
      // シュートを撃つ
      if (receiver_ids_.size() == 0) {
        Eigen::Vector2d target = {enemygoal_x, 0 * ballysign};
        const double power     = 255;
        kick_->kick_to(shoot_pos.x(), shoot_pos.y());
        kick_->set_dribble(0);
        kick_->set_mode(action::kick_action::mode::ball);
        kick_->set_angle_margin(0.15);
        kick_->set_kick_type({model::command::kick_type_t::line, power});
        baseaction.push_back(kick_);
      } else {
        free_robots_.push_back(kicker_id_);
      }
      for (auto receiver_id : receiver_ids_) {
        if (receiver_id == shooter_id_ && !try_direct) {
          const double power = 255;
          get_ball_->set_target(shoot_pos.x(), shoot_pos.y());
          get_ball_->set_pow(power);
          baseaction.push_back(get_ball_);
        } else {
          if (free_robots_.size() < 2) free_robots_.push_back(receiver_id);
        }
      }
      break;
    }
    case state::finished: {
      // 終了
      free_robots_.push_back(kicker_id_);
      for (auto receiver_id : receiver_ids_) {
        free_robots_.push_back(receiver_id);
      }
      break;
    }
  }
  return baseaction;
}
bool setplay::finished() {
  return state_ == state::finished;
}
// レシーバを選ぶ関数{{{
int setplay::chose_location(std::vector<Eigen::Vector2d> targets,
                            model::world::robots_list enemy_robots, int dist) {
  const auto& ball = world_.ball();
  int result       = 0;
  double max_dist  = 0;
  int cnt          = 0;
  for (auto& target : targets) {
    double min_dist = std::numeric_limits<double>::max();
    for (auto& enemy_robot_p : enemy_robots) {
      auto& enemy_robot = std::get<1>(enemy_robot_p);
      // ボールと目的位置の間に敵ロボットがいるかしらべたかったがこれだとがばがばなので要修正
      if (/* (ball.x() - enemy_robot.x()) * (target.x() - enemy_robot.x()) < 0 &&
 */ (ball.y() - enemy_robot.y()) *
              (target.y() - enemy_robot.y()) <
          0) {
        const Eigen::Vector3d to_target(target.x() - ball.x(), target.y() - ball.y(), 0);
        const Eigen::Vector3d to_enemy(enemy_robot.x() - ball.x(), enemy_robot.y() - ball.y(),
                                       0);
        const double distance = std::abs(to_target.cross(to_enemy).norm() / to_target.norm());
        if (distance < dist) {
          return std::numeric_limits<int>::max();
        }
        if (distance < min_dist) {
          min_dist = distance;
        }
      }
    }
    if (min_dist > max_dist) {
      max_dist = min_dist;
      result   = cnt;
    }
    cnt++;
  }
  return result;
} // }}}
// ベクトルを渡すと角度を返してくれるもの{{{
double setplay::vectorangle(Eigen::Vector2d vec) {
  return std::atan2(vec.y(), vec.x());
} // }}}
} // namespace agent
} // namespace game
} // namespace ai_server
