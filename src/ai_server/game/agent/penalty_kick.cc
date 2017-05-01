#include "penalty_kick.h"
#include "ai_server/model/command.h"
#include "ai_server/util/math.h"

#include <cmath>
#include <boost/math/constants/constants.hpp>

namespace ai_server {
namespace game {
namespace agent {

penalty_kick::penalty_kick(const model::world& world, bool is_yellow, unsigned int kicker_id,
                           const std::vector<unsigned int>& ids)
    : base(world, is_yellow),
      move_(std::make_shared<action::move>(world, is_yellow, kicker_id)),
      kick_(std::make_shared<action::kick_action>(world, is_yellow, kicker_id)) {
  kicker_id_  = kicker_id;
  start_flag_ = false;
  kick_x_     = 0;
  kick_y_     = 0;
  kick_theta_ = 0;
  target_x_   = 0;
  target_y_   = 0;
}

bool penalty_kick::start_flag() const {
  return start_flag_;
}

void penalty_kick::set_start_flag(bool start_flag) {
  start_flag_ = start_flag;
}

std::vector<std::shared_ptr<action::base>> penalty_kick::execute() {
  calculate();
  if (!exe_.empty()) {
    exe_.pop_back();
  }

  if (!start_flag_) {
    if (!move_->finished()) {
      //移動
      move_->move_to(kick_x_, kick_y_, kick_theta_);
      exe_.push_back(move_);
    } else {
      //何もしない
      auto nop = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
      exe_.push_back(nop);
    }
  } else {
    if (!kick_->finished()) {
      //キック
      kick_->kick_to(target_x_, target_y_);
      model::command::kick_flag_t kick_flag(model::command::kick_type_t::line, 30.0);
      kick_->set_kick_type(kick_flag);
      exe_.push_back(kick_);
    } else {
      //何もしない
      auto nop = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
      exe_.push_back(nop);
    }
  }
  return exe_;
}

void penalty_kick::calculate() {
  using namespace boost::math::constants;
  const auto goal_x = world_.field().x_max();
  const auto goal_y = 0;
  const auto ball_x = world_.ball().x();
  const auto ball_y = world_.ball().y();

  target_x_ = goal_x;
  if (target_y_ == 0) {
    target_y_ = ball_y < 0 ? goal_y + 400 : goal_y - 400;
  }
  // PK待機位置
  //パラメータ計算
  double b = 1;
  double a = (target_y_ - ball_y) / (target_x_ - ball_x);
  double c = -ball_y + (-a * ball_x);
  double l = a * a + b * b;
  double k = a * ball_x + b * ball_y + c;
  double d = l * 500 * 500 - k * k;
  //位置計算
  if (d > 0) {
    kick_x_ = (ball_x - a / l * k) - (b / l * std::sqrt(d));
  } else if (std::abs(d) < 0.0000000001) {
    kick_x_ = ball_x - a * k / l;
  } else {
    kick_x_ = ball_x - 500;
  }
  kick_y_ = (ball_y < 0)
                ? -std::sqrt((500 * 500) - (kick_x_ - ball_x) * (kick_x_ - ball_x)) + ball_y
                : (std::sqrt((500 * 500) - (kick_x_ - ball_x) * (kick_x_ - ball_x)) + ball_y);
  kick_theta_ = std::atan2(target_y_ - ball_y, target_x_ - ball_x);
}
}
}
}