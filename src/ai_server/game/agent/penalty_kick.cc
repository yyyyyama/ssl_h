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
      time_flag_(true),
      kicker_id_(kicker_id),
      kick_x_(0),
      kick_y_(0),
      kick_theta_(0),
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
      calculate_kick_position(550, 0);
      auto move = std::make_shared<action::move>(world_, is_yellow_, kicker_id_);
      move->move_to(kick_x_, kick_y_, kick_theta_);
      exe.push_back(move);
      if (move->finished()) {
        auto nop = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
        exe.push_back(nop);
      }
    } else {
      if (time_flag_) {
        change_command_time_ = point;
        time_flag_           = false;
      }
      if (rush_ && start_rush(ball) || time_over(point, 6)) {
        exe.push_back(rush_);
      } else {
        rush_.reset();
        rush_ = std::make_shared<action::rush>(world_, is_yellow_, kicker_id_);
        calculate_kick_position(180);
        rush_move_ = std::make_shared<action::move>(world_, is_yellow_, kicker_id_);
        rush_move_->move_to(kick_x_, kick_y_, kick_theta_);
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
void penalty_kick::calculate_kick_position(double keep_out, double theta) {
  using boost::math::constants::pi;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();

  kick_theta_ = 0;
  kick_x_     = ball.x() - keep_out;
  kick_y_     = 0;
}

void penalty_kick::calculate_kick_position(double keep_out) {
  using boost::math::constants::pi;
  const auto ball  = world_.ball();
  const auto omega = pi<double>() / 64;

  if (kick_theta_ > atan2(2, 2)) {
    kick_theta_ -= omega;
  } else if (kick_theta_ < -atan2(2, 2)) {
    kick_theta_ += omega;
  } else {
    kick_theta_ = prev_dec_ ? kick_theta_ - omega : kick_theta_ + omega;
  }
  prev_dec_ = kick_theta_ - prev_kick_theta_ < 0;
  kick_x_   = ball.x() - keep_out;
  kick_y_   = -keep_out * std::sin(pi<double>() - kick_theta_);
  kick_y_ += ball.y();
  prev_kick_theta_ = kick_theta_;
}

bool penalty_kick::start_rush(model::ball ball) {
  const auto enemies    = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto x          = our_robots.at(kicker_id_).x();
  const auto y          = our_robots.at(kicker_id_).y();

  const auto a        = (ball.x() - x) / (ball.y() - y);
  const auto b        = x - a * y;
  const auto target_x = world_.field().x_max();
  const auto target_y = 1000 * tan(kick_theta_);

  if (!enemies.count(keeper_id_)) {
    return true;
  }
  const auto keeper_y = enemies.at(keeper_id_).y();
  if (std::abs(keeper_y - target_y) > 400 && std::fabs(target_y) < 350 &&
      std::hypot((x - ball.x()), y - ball.y()) < 300) {
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
