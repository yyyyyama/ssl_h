#include <cmath>
#include <map>

#include "ai_server/game/action/move.h"
#include "ai_server/game/agent/defense.h"
#include "ai_server/model/command.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/geometry.h"
#include "ai_server/util/math/to_vector.h"

namespace ai_server {
namespace game {
namespace agent {

defense::defense(context& ctx, unsigned int keeper_id,
                 const std::vector<unsigned int>& wall_ids,
                 const std::vector<unsigned int>& marking_ids)
    : base(ctx),
      keeper_id_(keeper_id),
      wall_ids_(wall_ids),
      marking_ids_(marking_ids),
      mode_(defense_mode::normal_mode),
      ball_(Eigen::Vector2d::Zero()) {
  // actionの生成部分
  //
  // actionの初期化は一回しか行わない
  //
  //

  //キーパー用のaction
  keeper_     = make_action<action::goal_keep>(keeper_id_);
  keeper_get_ = make_action<action::get_ball>(keeper_id_);
}

void defense::set_mode(defense_mode mode) {
  mode_ = mode;
}

std::vector<unsigned int> defense::marking_ids() const {
  return marking_ids_re_;
}

std::vector<std::shared_ptr<action::base>> defense::execute() {
  using boost::math::constants::pi;

  std::vector<std::shared_ptr<action::base>> baseaction;
  //ボールの座標
  const Eigen::Vector2d ball_vel = util::math::velocity(world().ball());
  // visionから取得した今のボールの位置
  const Eigen::Vector2d ball_pos(std::max(world().ball().x(), world().field().x_min()),
                                 world().ball().y());
  const Eigen::Vector2d ball_k(ball_vel * 0.5);
  // 0.5s後のボールの位置(推定)
  const Eigen::Vector2d ball(ball_pos + ball_k);

  //状態を遷移させるためのボールの位置
  if (ball_vel.norm() < 150.0) {
    ball_ = ball_pos;
  }

  //ゴールの座標
  const Eigen::Vector2d goal(world().field().x_min(), 0.0);

  const auto ally_robots  = model::our_robots(world(), team_color());
  const auto enemy_robots = model::enemy_robots(world(), team_color());

  //ここから壁の処理
  if (!wall_ids_.empty()) {
    std::vector<unsigned int> active_walls;
    std::copy_if(wall_ids_.begin(), wall_ids_.end(), std::back_inserter(active_walls),
                 [&](const unsigned int x) { return ally_robots.count(x); });

    //実際にアクションを詰めて返す
    std::vector<Eigen::Vector2d> enemy_pos;
    std::transform(enemy_robots.begin(), enemy_robots.end(), std::back_inserter(enemy_pos),
                   [](const auto& eit) { return util::math::position(eit.second); });

    std::sort(enemy_pos.begin(), enemy_pos.end(), [&goal](const auto& a, const auto& b) {
      return (a - goal).norm() < (b - goal).norm();
    });

    std::vector<Eigen::Vector2d> target_pos;
    target_pos.push_back(ball);
    auto find = std::find_if(enemy_pos.begin(), enemy_pos.end(),
                             [&ball](const auto& it) { return (it - ball).norm() > 300; });
    if (find != enemy_pos.end()) target_pos.push_back(*find);

    //取り出した値をソートする
    if (wall_ids_.size() != 1) {
      std::sort(target_pos.begin(), target_pos.end(), [&goal](const auto& a, const auto& b) {
        return util::math::wrap_to_2pi(std::atan2(a.y() - goal.y(), a.x() - goal.x()) +
                                       pi<double>() / 2.0) <
               util::math::wrap_to_2pi(std::atan2(b.y() - goal.y(), b.x() - goal.x()) +
                                       pi<double>() / 2.0);
      });
    }

    wall_pairs_.clear();
    for (const auto& target_it : target_pos) {
      const auto itr = std::min_element(
          active_walls.cbegin(), active_walls.cend(),
          [&target_it, &ally_robots, &goal](const auto& a, const auto& b) {
            return std::abs(util::math::wrap_to_pi(
                       std::atan2(ally_robots.at(a).y() - goal.y(),
                                  ally_robots.at(a).x() - goal.x()) -
                       std::atan2(target_it.y() - goal.y(), target_it.x() - goal.x()))) <
                   std::abs(util::math::wrap_to_pi(
                       std::atan2(ally_robots.at(b).y() - goal.y(),
                                  ally_robots.at(b).x() - goal.x()) -
                       std::atan2(target_it.y() - goal.y(), target_it.x() - goal.x())));
          });
      if (itr == active_walls.end()) {
        break;
      }

      wall_pairs_[*itr] = target_it;
      active_walls.erase(itr);
    }

    Eigen::Vector2d last_pos{0.0, 0.0};
    for (auto wall_it = wall_pairs_.begin(); wall_it != wall_pairs_.end(); ++wall_it) {
      int i          = static_cast<int>(std::distance(wall_pairs_.begin(), wall_it));
      auto guard     = make_action<action::guard>((*wall_it).first);
      const auto pos = (*wall_it).second;
      if (i == 0) last_pos = pos;
      const double last_theta = std::atan2(last_pos.y() - goal.y(), last_pos.x() - goal.x());
      const double pos_theta  = std::atan2(pos.y() - goal.y(), pos.x() - goal.x());
      //目標が同じなら軌道をずらす
      bool shift_flag = i == 1 && std::abs(util::math::wrap_to_pi(pos_theta - last_theta)) <
                                      pi<double>() / 12;
      guard->move_on(shift_flag);
      guard->move_to(pos.x(), pos.y());
      baseaction.push_back(guard);
    }

    if (!active_walls.empty()) {
      const auto wf = world().field();
      std::vector<Eigen::Vector2d> pos_candidates{
          {wf.x_min() + wf.penalty_length() + 1000.0, 300.0},
          {wf.x_min() + wf.penalty_length() + 1000.0, -300.0},
          {wf.x_min() + wf.penalty_length() + 1000.0, 600.0},
          {wf.x_min() + wf.penalty_length() + 1000.0, -600.0},
          {wf.x_min() + wf.penalty_length() + 1000.0, 900.0},
          {wf.x_min() + wf.penalty_length() + 1000.0, -900.0},
          {wf.x_min() + wf.penalty_length() + 1000.0, 1200.0},
          {wf.x_min() + wf.penalty_length() + 1000.0, -1200.0},
          {wf.x_min() + wf.penalty_length() + 1000.0, 1500.0},
          {wf.x_min() + wf.penalty_length() + 1000.0, -1500.0},
          {wf.x_min() + wf.penalty_length() + 1000.0, 1800.0}};
      for (const auto id : active_walls) {
        const auto itr =
            std::min_element(pos_candidates.cbegin(), pos_candidates.cend(),
                             [robot_pos = util::math::position(ally_robots.at(id))](
                                 const auto& a, const auto& b) {
                               return (robot_pos - a).norm() < (robot_pos - b).norm();
                             });
        auto move = make_action<action::move>(id);
        move->move_to(*itr, 0.0);
        baseaction.push_back(move);
        pos_candidates.erase(itr);
      }
    }
  }
  //ここからキーパーの処理
  //
  //
  //
  Eigen::Vector2d keeper(Eigen::Vector2d::Zero());
  if (ally_robots.count(keeper_id_) == 1) {
    const auto demarcation1 = 1000.0; //クリアする範囲
    const auto demarcation2 = 400.0;  //クリアする範囲
    bool get_flag           = (ball_ - goal).norm() < demarcation1 &&
                    (ball_ - goal).norm() > demarcation2 && ball_vel.norm() < 500.0 &&
                    mode_ != defense_mode::stop_mode;

    if (get_flag) {
      keeper_get_->set_chip(true);
      baseaction.push_back(keeper_get_); //配列を返すためにキーパーを統合する
    } else {
      baseaction.push_back(keeper_); //配列を返すためにキーパーを統合する
    }
  }
  return baseaction;
}
} // namespace agent
} // namespace game
} // namespace ai_server
