#include <algorithm>
#include <cmath>
#include <cfloat>
#include <boost/math/constants/constants.hpp>

#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"
#include "ai_server/game/action/vec.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/rrt_star.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "ball_placement.h"

using boost::math::constants::pi;
using namespace std::chrono_literals;

namespace ai_server {
namespace game {
namespace agent {

ball_placement::ball_placement(context& ctx, const std::vector<unsigned int>& ids,
                               const Eigen::Vector2d target, bool is_active)
    : base(ctx),
      ids_(ids),
      visible_ids_(ids),
      lost_count_(3s),
      abp_target_(target),
      is_active_(is_active) {
  const auto our_robots = model::our_robots(world(), team_color());
  const auto ball       = world().ball();
  const Eigen::Vector2d ball_pos(ball.x(), ball.y());
  const Eigen::Vector2d ball_vel(ball.vx(), ball.vy());
  for (const auto& id : ids_) {
    abp_[id]       = make_action<action::autonomous_ball_place>(id, abp_target_);
    receive_[id]   = make_action<action::receive>(id);
    move_[id]      = make_action<action::move>(id);
    robot_pos_[id] = our_robots.count(id)
                         ? Eigen::Vector2d(our_robots.at(id).x(), our_robots.at(id).y())
                         : Eigen::Vector2d(0, 0);
    robot_vel_[id] = our_robots.count(id)
                         ? Eigen::Vector2d(our_robots.at(id).vx(), our_robots.at(id).vy())
                         : Eigen::Vector2d(0, 0);
    lost_point_[id] = std::chrono::steady_clock::now();
    is_lost_[id]    = false;
  }
  auto tmp_ids = ids;
  if (static_cast<int>(ids_.size()) > 0) {
    const auto chaser_itr = std::min_element(ids_.cbegin(), ids_.cend(), [&](auto& a, auto& b) {
      return ((robot_pos_[a] + robot_vel_[a]) - (ball_pos + ball_vel)).norm() <
             ((robot_pos_[b] + robot_vel_[b]) - (ball_pos + ball_vel)).norm();
    });
    chaser_               = *chaser_itr;
  }
  auto end =
      std::remove_if(tmp_ids.begin(), tmp_ids.end(), [this](auto& i) { return i == chaser_; });
  tmp_ids.erase(end, tmp_ids.end());
  if (static_cast<int>(tmp_ids.size()) > 0) {
    const auto receiver_itr =
        std::min_element(tmp_ids.cbegin(), tmp_ids.cend(), [this](auto& a, auto& b) {
          return (robot_pos_[a] - abp_target_).norm() < (robot_pos_[b] - abp_target_).norm();
        });
    current_receiver_ = *receiver_itr;
  }
}

void ball_placement::set_target(const Eigen::Vector2d target) {
  abp_target_ = target;
  for (const auto& id : ids_) {
    abp_[id] = make_action<action::autonomous_ball_place>(id, abp_target_);
  }
}

void ball_placement::set_active(const bool is_active) {
  is_active_ = is_active;
}

std::vector<std::shared_ptr<action::base>> ball_placement::execute() {
  std::vector<std::shared_ptr<action::base>> baseaction;
  if (static_cast<int>(ids_.size() == 0)) return baseaction;
  const auto wf   = world().field();
  const auto ball = world().ball();
  const Eigen::Vector2d ball_pos(ball.x(), ball.y());
  const Eigen::Vector2d ball_vel(ball.vx(), ball.vy());
  const auto our_robots = model::our_robots(world(), team_color());
  const auto ene_robots = model::enemy_robots(world(), team_color());
  std::vector<unsigned int> our_ids;
  for (const auto& our : our_robots) our_ids.push_back(our.first);
  std::vector<unsigned int> ene_ids;
  for (const auto& ene : ene_robots) ene_ids.push_back(ene.first);

  ///////////////////////////////////////////
  // lost判定 ///////////////////////////////
  for (const auto& id : ids_) {
    if (our_robots.count(id)) {
      robot_pos_[id]  = Eigen::Vector2d(our_robots.at(id).x(), our_robots.at(id).y());
      robot_vel_[id]  = Eigen::Vector2d(our_robots.at(id).vx(), our_robots.at(id).vy());
      lost_point_[id] = std::chrono::steady_clock::now();
    } else {
      std::chrono::duration<double> time = std::chrono::steady_clock::now() - lost_point_[id];
      robot_pos_[id]                     = robot_pos_[id] + robot_vel_[id] * time.count();
    }
    is_lost_[id] = (std::chrono::steady_clock::now() - lost_point_[id]) > lost_count_;
    auto new_end = std::remove_if(visible_ids_.begin(), visible_ids_.end(),
                                  [&id](unsigned int i) { return i == id; });
    visible_ids_.erase(new_end, visible_ids_.end());
    visible_ids_.shrink_to_fit();
    if (!is_lost_[id]) visible_ids_.push_back(id);
  }
  if (static_cast<int>(visible_ids_.size()) == 0) return baseaction;

  // これ以上の距離ならパス
  constexpr double pass_threshold = 3000.0;
  const bool pass_flag = is_active_ && (abp_target_ - ball_pos).norm() > pass_threshold &&
                         static_cast<int>(visible_ids_.size()) > 1;

  ///////////////////////////////////////////
  // 役割の更新 /////////////////////////////
  {
    auto tmp_ids = visible_ids_;
    // chaser ///////////////////////////////
    // TODO: chaserをいい感じに決める
    if (is_active_) {
      std::vector<std::tuple<unsigned int, double>> score_list;
      for (const auto& id : tmp_ids) {
        double s      = 0.0;
        const auto br = robot_pos_[id] - ball_pos;
        s -= br.norm();
        score_list.push_back({id, s});
      }
      const auto chaser_itr = std::max_element(
          score_list.cbegin(), score_list.cend(),
          [&](const auto& a, const auto& b) { return std::get<1>(a) < std::get<1>(b); });
      chaser_        = std::get<0>(*chaser_itr);
      const auto end = std::remove_if(tmp_ids.begin(), tmp_ids.end(),
                                      [this](const auto& i) { return i == chaser_; });
      tmp_ids.erase(end, tmp_ids.end());
      tmp_ids.shrink_to_fit();
    }

    // receiver /////////////////////////////
    if (is_active_ && static_cast<int>(tmp_ids.size()) > 0 && pass_flag) {
      const auto receiver_itr =
          std::min_element(tmp_ids.cbegin(), tmp_ids.cend(), [this](auto& a, auto& b) {
            return (robot_pos_[a] - abp_target_).norm() < (robot_pos_[b] - abp_target_).norm();
          });
      current_receiver_ = *receiver_itr;
      const auto end    = std::remove_if(tmp_ids.begin(), tmp_ids.end(),
                                      [this](const auto& i) { return i == current_receiver_; });
      tmp_ids.erase(end, tmp_ids.end());
      tmp_ids.shrink_to_fit();
    }

    // waiter ///////////////////////////////
    { waiters_ = tmp_ids; }
  }

  /////////////////////////////////////////
  // 動作を設定 ///////////////////////////
  {
    // chaser /////////////////////////////
    if (is_active_ && our_robots.count(chaser_)) {
      const unsigned int id           = chaser_;
      const Eigen::Vector2d robot_pos = robot_pos_[id];
      // 待機位置からの許容距離
      constexpr double receiver_margin = 200.0;
      if (ball_vel.dot((robot_pos - ball_pos).normalized()) > 500.0) {
        receive_[id]->set_dribble(5);
        baseaction.push_back(receive_[id]);
      } else if (std::abs(ball_pos.x()) > wf.x_max() || std::abs(ball_pos.y()) > wf.y_max() ||
                 !pass_flag || !our_robots.count(current_receiver_) ||
                 (robot_pos_[current_receiver_] - abp_target_).norm() > receiver_margin) {
        abp_[id]->set_kick_type({model::command::kick_type_t::none, 0});
        baseaction.push_back(abp_[id]);
      } else if (pass_flag &&
                 (robot_pos_[current_receiver_] - abp_target_).norm() < receiver_margin) {
        abp_[id]->set_kick_type({model::command::kick_type_t::line, 50});
        baseaction.push_back(abp_[id]);
      } else {
        waiters_.push_back(id);
      }
    }

    // receiver & waiter ///////////////////////////
    // TODO: 経路等の設定をしてplacementの邪魔をしないようにする
    if (is_active_ && pass_flag) waiters_.push_back(current_receiver_);
    int count = 0;
    for (const auto& id : waiters_) {
      if (!our_robots.count(id)) continue;
      const bool is_receiver          = is_active_ && id == current_receiver_ && pass_flag;
      const Eigen::Vector2d robot_pos = robot_pos_[id];
      const Eigen::Vector2d base_point(wf.x_min() + wf.penalty_length() + 1000.0, 0.0);
      // 待機位置の間隔
      constexpr double wait_diff = 350.0;
      const double theta =
          std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x());
      const Eigen::Vector2d target_pos =
          is_receiver
              ? static_cast<Eigen::Vector2d>(
                    abp_target_ - 100.0 * Eigen::Vector2d(std::cos(theta), std::sin(theta)))
              : base_point + wait_diff * (count % 2 == 0 ? 1.0 : -1.0) *
                                 Eigen::Vector2d(0, count / 2 + count % 2);
      // 待機する
      std::unique_ptr<planner::rrt_star> rrt = std::make_unique<planner::rrt_star>();
      rrt->set_area(wf, 200.0);
      {
        planner::obstacle_list obstacles;
        obstacles.add(model::obstacle::enemy_penalty_area(wf, 150.0));
        obstacles.add(model::obstacle::our_penalty_area(wf, 150.0));

        for (const auto& oid : our_ids) {
          if (oid != id && our_robots.count(oid))
            obstacles.add(
                model::obstacle::point{util::math::position(our_robots.at(oid)), 200.0});
        }
        for (const auto& eid : ene_ids) {
          if (ene_robots.count(eid))
            obstacles.add(
                model::obstacle::point{util::math::position(ene_robots.at(eid)), 200.0});
        }

        if (!is_receiver) {
          obstacles.add(model::obstacle::segment{{abp_target_, ball_pos}, 500.0});
        }

        rrt->set_node_count(10);
        rrt->set_max_branch_length(50.0);
      }
      move_[id]->set_path_planner(std::move(rrt));
      // TODO: actionに障害物リストを渡す
      move_[id]->move_to(target_pos, theta);
      baseaction.push_back(move_[id]);
      count++;
    }
  }
  return baseaction;
}
} // namespace agent
} // namespace game
} // namespace ai_server
