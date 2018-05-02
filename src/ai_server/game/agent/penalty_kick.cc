#include "penalty_kick.h"
#include "ai_server/model/command.h"
#include "ai_server/util/math.h"

#include <cmath>
#include <boost/math/constants/constants.hpp>

namespace ai_server {
namespace game {
namespace agent {

penalty_kick::penalty_kick(const model::world& world, bool is_yellow, unsigned int kicker_id,
                           const std::vector<unsigned int>& ids, unsigned int enemy_keeper)
    : base(world, is_yellow),
      mode_(penalty_kick::penalty_mode::defense),
      start_flag_(false),
      ids_(ids),
      move_(std::make_shared<action::move>(world_, is_yellow_, kicker_id_)),
      rush_(std::make_shared<action::rush>(world_, is_yellow_, kicker_id_)),
      initial_flag_(true),
      shoot_count_(6),
      setted_ball_(world_.ball()),
      kicker_id_(kicker_id),
      kick_x_(0),
      kick_y_(0),
      kick_theta_(0),
      theta_(0),
      keeper_id_(enemy_keeper) {}

penalty_kick::penalty_mode penalty_kick::mode() {
  return mode_;
}

void penalty_kick::set_mode(penalty_kick::penalty_mode mode) {
  mode_ = mode;
}

bool penalty_kick::start_flag() const {
  return start_flag_;
}

void penalty_kick::set_start_flag(bool start_flag) {
  start_flag_ = start_flag;
}

bool penalty_kick::finished() {
  return rush_->finished();
}

std::vector<std::shared_ptr<action::base>> penalty_kick::execute() {
  //////////////////////////////
  //      キッカーの処理
  /////////////////////////////
  using boost::math::constants::pi;
  std::vector<std::shared_ptr<action::base>> exe;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  std::vector<unsigned int> visible_robots;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_robots),
               [&our_robots](auto id) { return our_robots.count(id); });
  const auto ball  = world_.ball();
  const auto point = std::chrono::high_resolution_clock::now();

  //攻撃側が指定されているとき
  if (mode_ == penalty_kick::penalty_mode::attack && our_robots.count(kicker_id_)) {
    if (!start_flag_) {
      kick_x_     = ball.x() - 550;
      kick_y_     = 0;
      kick_theta_ = 0;
      auto move   = std::make_shared<action::move>(world_, is_yellow_, kicker_id_);
      move->move_to(kick_x_, kick_y_, kick_theta_);
      exe.push_back(move);
      if (move->finished()) {
        auto nop = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
        exe.push_back(nop);
      }
    } else {
      if (initial_flag_) {
        change_command_time_ = point;
        setted_ball_         = world_.ball();
        setted_robot_        = our_robots.at(kicker_id_);
        initial_flag_        = false;
        rush_move_           = std::make_shared<action::move>(world_, is_yellow_, kicker_id_);
        calculate_kick_position(200);
        rush_move_->move_to(kick_x_, kick_y_, kick_theta_);
      }
      if ((rush_ && start_rush(ball)) || time_over(point, shoot_count_)) {
        if (std::hypot((setted_ball_.x() - ball.x()), setted_ball_.y() - ball.y()) < 30 ||
            std::hypot((setted_robot_.x() - our_robots.at(kicker_id_).x()),
                       setted_robot_.y() - our_robots.at(kicker_id_).y()) < 50) {
          exe.push_back(rush_);
        } else {
          auto nop = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
          exe.push_back(nop);
          shoot_count_ = 0;
        }
      } else {
        rush_.reset();
        rush_ = std::make_shared<action::rush>(world_, is_yellow_, kicker_id_);
        if (std::hypot(our_robots.at(kicker_id_).x() - kick_x_,
                       our_robots.at(kicker_id_).y() - kick_y_) < 30 &&
            std::fabs(kick_theta_ - our_robots.at(kicker_id_).theta()) < pi<double>() / 6) {
          calculate_kick_position(200);
          rush_move_->move_to(kick_x_, kick_y_, kick_theta_);
        }
        exe.push_back(rush_move_);
      }
    }
  }

  //////////////////////////////
  //      キッカー以外の処理
  //////////////////////////////
  //パラメータ設定
  int count       = 2;   //何番目のロボットか判別
  double interval = 0.0; //ロボットの間隔
  double line     = 0.0; //移動位置の基準
  if (world_.ball().x() < 0) {
    line     = -2000;
    interval = 500;
  } else {
    line     = 2000;
    interval = -500;
  }

  //ちょうどいい位置に並べる
  for (auto it = visible_robots.begin(); it != visible_robots.end(); it++, count++) {
    double x = line + interval * (count / 2);
    double y = 0;
    if (count % 2) { //順番に左右に分ける
      y = world_.field().y_min() + 500;
    } else {
      y = world_.field().y_max() - 500;
    }
    auto move = std::make_shared<action::move>(world_, is_yellow_, *it);
    move->move_to(x, y, 0);
    exe.push_back(move);
  }

  return exe;
}

// PK待機位置の計算
void penalty_kick::calculate_kick_position(double keep_out) {
  using boost::math::constants::pi;
  const auto ball  = world_.ball();
  const auto omega = pi<double>() / 32;

  if (theta_ > std::atan2(2, 2)) {
    theta_ -= omega;
  } else if (theta_ < -std::atan2(2, 2)) {
    theta_ += omega;
  } else {
    theta_ = prev_dec_ ? theta_ - omega : theta_ + omega;
  }
  prev_dec_   = theta_ - prev_theta_ < 0;
  kick_x_     = ball.x() - keep_out * std::cos(theta_);
  kick_y_     = ball.y() - keep_out * std::sin(theta_);
  prev_theta_ = theta_;
  kick_theta_ = std::atan2(ball.y() - kick_y_, ball.x() - kick_x_);
}

bool penalty_kick::start_rush(model::ball ball) {
  using boost::math::constants::pi;
  const auto enemies    = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto x          = our_robots.at(kicker_id_).x();
  const auto y          = our_robots.at(kicker_id_).y();
  const auto t          = our_robots.at(kicker_id_).theta();

  const auto a        = (ball.y() - y) / (ball.x() - x);
  const auto b        = y - a * x;
  const auto target_x = world_.field().x_max();
  const auto target_y = a * target_x + b;

  if (!enemies.count(keeper_id_)) {
    return true;
  }
  const auto keeper_y = enemies.at(keeper_id_).y();
  if (std::abs(keeper_y - target_y) > 400 && std::fabs(target_y) < 400 &&
      std::hypot((x - ball.x()), y - ball.y()) < 300 &&
      std::fabs(theta_ - t) < pi<double>() / 12) {
    return true;
  }
  return false;
}

bool penalty_kick::time_over(std::chrono::high_resolution_clock::time_point point, int count) {
  return (point - change_command_time_) > std::chrono::seconds(count);
}

} // namespace agent
} // namespace game
} // namespace ai_server
