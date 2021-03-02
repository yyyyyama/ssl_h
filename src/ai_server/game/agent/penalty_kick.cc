#include <cmath>
#include <boost/math/constants/constants.hpp>

#include "ai_server/game/action/with_planner.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/human_like.h"
#include "ai_server/util/math/to_vector.h"

#include "penalty_kick.h"

namespace ai_server::game::agent {

penalty_kick::penalty_kick(context& ctx, unsigned int kicker_id,
                           const std::vector<unsigned int>& ids, unsigned int enemy_keeper)
    : base(ctx),
      state_(attack_state::wait),
      mode_(penalty_kick::penalty_mode::defense),
      start_flag_(false),
      ids_(ids),
      turn_kick_(make_action<action::turn_kick>(kicker_id)),
      kicker_move_(make_action<action::move>(kicker_id)),
      kicker_id_(kicker_id),
      enemy_keeper_id_(enemy_keeper),
      change_command_time_(std::chrono::seconds(0)) {}

void penalty_kick::set_mode(penalty_kick::penalty_mode mode) {
  mode_ = mode;
}

void penalty_kick::set_start_flag(bool start_flag) {
  start_flag_ = start_flag;
}

bool penalty_kick::finished() const {
  return turn_kick_->finished();
}

std::vector<std::shared_ptr<action::base>> penalty_kick::execute() {
  std::vector<std::shared_ptr<action::base>> exe;
  const auto our_robots                  = model::our_robots(world(), team_color());
  const auto enemy_robots                = model::enemy_robots(world(), team_color());
  const auto& enemy_keeper               = enemy_robots.at(enemy_keeper_id_);
  const Eigen::Vector2d enemy_keeper_pos = util::math::position(enemy_keeper);
  const auto ball                        = util::math::position(world().ball());
  const auto field                       = world().field();
  const auto penalty_mark                = field.back_penalty_mark();
  const auto point                       = std::chrono::steady_clock::now();

  using boost::math::constants::pi;

  // 視認可能なロボットを抽出する
  std::vector<unsigned int> visible_ids;
  std::copy_if(
      ids_.cbegin(), ids_.cend(), std::back_inserter(visible_ids),
      [&our_robots, this](auto id) { return id != kicker_id_ && our_robots.count(id); });

  // ロボットの障害物としての半径
  constexpr double obs_robot_rad = 200.0;
  // フィールド外に出られる距離
  constexpr double area_margin = 200.0;
  // 障害物設定
  planner::obstacle_list common_obstacles;
  common_obstacles.add(model::obstacle::our_penalty_area(field, 150.0));
  for (const auto& robot : enemy_robots) {
    common_obstacles.add(
        model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
  }

  //////////////////////////////
  //      キッカーの処理
  /////////////////////////////
  //攻撃側が指定されているとき
  if (mode_ == penalty_kick::penalty_mode::attack && our_robots.count(kicker_id_)) {
    auto hl = std::make_unique<planner::human_like>();
    hl->set_area(field, area_margin);
    auto obstacles = common_obstacles;
    for (const auto& robot : our_robots) {
      if (robot.first != kicker_id_)
        obstacles.add(
            model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    switch (state_) {
      // wait - 待機位置に移動して待機
      case attack_state::wait:
        if (start_flag_ && kicker_move_->finished()) {
          state_               = attack_state::check;
          change_command_time_ = point;
        }
        kicker_move_->move_to(penalty_mark.x - 600.0, penalty_mark.y, 0.0);
        obstacles.add(model::obstacle::enemy_penalty_area(field, 150.0));
        obstacles.add(model::obstacle::point{ball, 500.0});
        exe.push_back(
            std::make_shared<action::with_planner>(kicker_move_, std::move(hl), obstacles));
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
        if (enemy_keeper_pos.y() < 0) {
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

        exe.push_back(
            std::make_shared<action::with_planner>(turn_kick_, std::move(hl), obstacles));
        break;

      // kick - キック
      case attack_state::kick:
        turn_kick_->set_kick(true);
        exe.push_back(
            std::make_shared<action::with_planner>(turn_kick_, std::move(hl), obstacles));
        break;
    }
  } else if (mode_ == penalty_kick::penalty_mode::defense && our_robots.count(kicker_id_)) {
    visible_ids.push_back(kicker_id_);
  }

  //////////////////////////////
  //      キッカー以外の処理
  //////////////////////////////
  if (visible_ids.empty()) return exe;
  //パラメータ設定
  int count             = 2;                               //何番目のロボットか判別
  const double interval = ball.x() < 0 ? 500.0 : -500.0;   //ロボットの間隔
  const double line     = ball.x() < 0 ? -2000.0 : 2000.0; //移動位置の基準

  //ちょうどいい位置に並べる
  for (const auto id : visible_ids) {
    auto hl = std::make_unique<planner::human_like>();
    hl->set_area(field, area_margin);
    auto obstacles = common_obstacles;
    for (const auto& robot : our_robots) {
      if (robot.first != id)
        obstacles.add(
            model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    obstacles.add(model::obstacle::enemy_penalty_area(field, 150.0));
    obstacles.add(model::obstacle::point{ball, 600.0});
    const double x = line + interval * (count / 2);
    const double y =
        (count % 2 == 1) ? field.y_min() + 500.0 : field.y_max() - 500.0; //順番に左右に分ける
    auto move = make_action<action::move>(id);
    move->move_to(x, y, 0);
    exe.push_back(std::make_shared<action::with_planner>(move, std::move(hl), obstacles));
    count++;
  }

  return exe;
}

} // namespace ai_server::game::agent
