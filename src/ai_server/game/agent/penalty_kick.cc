#include "penalty_kick.h"
#include <cmath>
#include <boost/math/constants/constants.hpp>

namespace ai_server {
namespace game {
namespace agent {

penalty_kick::penalty_kick(const model::world& world, bool is_yellow, unsigned int kicker_id,
                           const std::vector<unsigned int>& ids, unsigned int enemy_keeper)
    : base(world, is_yellow),
      state_(attack_state::wait),
      mode_(penalty_kick::penalty_mode::defense),
      start_flag_(false),
      ids_(ids),
      turn_kick_(std::make_shared<action::turn_kick>(world, is_yellow_, kicker_id)),
      kicker_move_(std::make_shared<action::move>(world, is_yellow_, kicker_id)),
      kicker_id_(kicker_id),
      keeper_id_(enemy_keeper) {}

penalty_kick::penalty_mode penalty_kick::mode() const {
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
  return turn_kick_->finished();
}

std::vector<std::shared_ptr<action::base>> penalty_kick::execute() {
  //////////////////////////////
  //      キッカーの処理
  /////////////////////////////
  std::vector<std::shared_ptr<action::base>> exe;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  std::vector<unsigned int> visible_ids;
  std::copy_if(
      ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
      [&our_robots, this](auto id) { return id != kicker_id_ && our_robots.count(id); });
  const auto ball  = world_.ball();
  const auto field = world_.field();

  //攻撃側が指定されているとき
  if (mode_ == penalty_kick::penalty_mode::attack && our_robots.count(kicker_id_)) {
    if (start_flag_ && kicker_move_->finished()) {
      state_ = attack_state::kick;
    }
    switch (state_) {
      case attack_state::wait:
        kicker_move_->move_to(field.x_max() - field.penalty_length() - 600, 0.0, 0.0);
        exe.push_back(kicker_move_);
        break;
      case attack_state::kick:
        exe.push_back(turn_kick_);
        break;
    }
  } else if (mode_ == penalty_kick::penalty_mode::defense && our_robots.count(kicker_id_)) {
    visible_ids.push_back(kicker_id_);
  }

  //////////////////////////////
  //      キッカー以外の処理
  //////////////////////////////
  //パラメータ設定
  int count             = 2;                           //何番目のロボットか判別
  const double interval = ball.x() < 0 ? 500 : -500;   //ロボットの間隔
  const double line     = ball.x() < 0 ? -2000 : 2000; //移動位置の基準

  //ちょうどいい位置に並べる
  for (const auto id : visible_ids) {
    const double x = line + interval * (count / 2);
    const double y =
        (count % 2 == 1) ? field.y_min() + 500 : field.y_max() - 500; //順番に左右に分ける
    const auto move = std::make_shared<action::move>(world_, is_yellow_, id);
    move->move_to(x, y, 0);
    exe.push_back(move);
    count++;
  }

  return exe;
}

} // namespace agent
} // namespace game
} // namespace ai_server
