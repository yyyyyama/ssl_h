#include "penalty_kick.h"
#include "ai_server/util/math/to_vector.h"
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
      keeper_id_(enemy_keeper),
      change_command_time_(std::chrono::seconds(0)) {}

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
  const auto enemy_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto& keeper      = enemy_robots.at(keeper_id_);
  const Eigen::Vector2d keeper_pos = util::math::position(keeper);
  const auto ball                  = world_.ball();
  const auto field                 = world_.field();
  const auto point                 = util::clock_type::now();

  using boost::math::constants::pi;

  //攻撃側が指定されているとき
  if (mode_ == penalty_kick::penalty_mode::attack && our_robots.count(kicker_id_)) {
    switch (state_) {
      // wait - 待機位置に移動して待機
      case attack_state::wait:
        if (start_flag_ && kicker_move_->finished()) {
          state_               = attack_state::check;
          change_command_time_ = point;
        }
        kicker_move_->move_to(field.x_max() - field.penalty_length() - 600.0, 0.0, 0.0);
        exe.push_back(kicker_move_);
        break;

      // check - 状況に合わせて指定位置を変更
      case attack_state::check:
        // 5秒経過したらキック
        if (change_command_time_ - point > std::chrono::seconds(5)) {
          state_ = attack_state::kick;
        }

        // 基準となる位置の指定
        turn_kick_->set_start_pos({ball.x() - 80.0, ball.y()});

        // キーパーに合わせて位置を指定
        if (keeper_pos.y() < 0) {
          turn_kick_->set_angle(pi<double>() / 7);
          turn_kick_->set_radius(90.0);
        } else {
          turn_kick_->set_angle(-pi<double>() / 7);
          turn_kick_->set_radius(90.0);
        }

        // 指定位置に辿り着いたらキック
        if (turn_kick_->wait()) {
          state_ = attack_state::kick;
        }
        exe.push_back(turn_kick_);
        break;

      // kick - キック
      case attack_state::kick:
        turn_kick_->set_kick(true);
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
  int count             = 2;                               //何番目のロボットか判別
  const double interval = ball.x() < 0 ? 500.0 : -500.0;   //ロボットの間隔
  const double line     = ball.x() < 0 ? -2000.0 : 2000.0; //移動位置の基準

  //ちょうどいい位置に並べる
  for (const auto id : visible_ids) {
    const double x = line + interval * (count / 2);
    const double y =
        (count % 2 == 1) ? field.y_min() + 500.0 : field.y_max() - 500.0; //順番に左右に分ける
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
