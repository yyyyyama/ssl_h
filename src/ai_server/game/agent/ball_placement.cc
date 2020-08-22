#include <algorithm>
#include <cmath>

#include <boost/math/constants/constants.hpp>

#include "ai_server/game/action/with_planner.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/human_like.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "ball_placement.h"

using boost::math::constants::pi;
using namespace std::chrono_literals;

namespace ai_server::game::agent {

ball_placement::ball_placement(context& ctx, const std::vector<unsigned int>& ids,
                               const Eigen::Vector2d& target, bool is_active)
    : base(ctx), ids_(ids), lost_count_(3s), abp_target_(target), is_active_(is_active) {
  const auto our_robots = model::our_robots(world(), team_color());
  const auto now        = std::chrono::steady_clock::now();
  for (auto id : ids_) {
    abp_[id]       = make_action<action::ball_place>(id, abp_target_);
    receive_[id]   = make_action<action::receive>(id);
    move_[id]      = make_action<action::move>(id);
    robot_pos_[id] = our_robots.count(id) ? util::math::position(our_robots.at(id))
                                          : Eigen::Vector2d::Zero();
    lost_point_[id] = now;
  }
}

void ball_placement::set_target(const Eigen::Vector2d& target) {
  abp_target_ = target;
  for (auto id : ids_) abp_[id] = make_action<action::ball_place>(id, abp_target_);
}

void ball_placement::set_active(bool is_active) {
  is_active_ = is_active;
}

std::vector<std::shared_ptr<action::base>> ball_placement::execute() {
  std::vector<std::shared_ptr<action::base>> baseaction;
  if (ids_.empty()) return baseaction;
  const auto wf                  = world().field();
  const Eigen::Vector2d ball_pos = util::math::position(world().ball());
  const Eigen::Vector2d ball_vel = util::math::velocity(world().ball());
  const auto our_robots          = model::our_robots(world(), team_color());
  const auto ene_robots          = model::enemy_robots(world(), team_color());

  ///////////////////////////////////////////
  // lost判定 ///////////////////////////////
  std::vector<unsigned int> visible_ids;
  const auto now = std::chrono::steady_clock::now();
  for (auto id : ids_) {
    if (our_robots.count(id)) {
      robot_pos_[id]  = util::math::position(our_robots.at(id));
      lost_point_[id] = now;
    }
    if (now - lost_point_.at(id) < lost_count_) visible_ids.push_back(id);
  }
  if (visible_ids.empty()) return baseaction;

  // これ以上の距離ならパス
  constexpr double pass_threshold = 3000.0;
  const bool pass_flag =
      is_active_ && (abp_target_ - ball_pos).norm() > pass_threshold && visible_ids.size() > 1;

  ///////////////////////////////////////////
  // 役割の更新 /////////////////////////////
  {
    auto tmp_ids = visible_ids;
    // chaser ///////////////////////////////
    if (is_active_) {
      std::vector<unsigned int> candidates;
      std::copy_if(tmp_ids.cbegin(), tmp_ids.cend(), std::back_inserter(candidates),
                   [this, &ball_pos, &ball_vel](auto id) {
                     const auto& robot_pos = robot_pos_.at(id);
                     return (robot_pos - abp_target_).norm() < 500.0 &&
                            ball_vel.dot((robot_pos - ball_pos).normalized()) > 1000.0 &&
                            std::abs(util::math::wrap_to_pi(
                                std::atan2(ball_vel.y(), ball_vel.x()) -
                                std::atan2(robot_pos.y() - ball_pos.y(),
                                           robot_pos.x() - ball_pos.x()))) < 0.1 * pi<double>();
                   });
      if (candidates.empty()) {
        chaser_ = *std::min_element(
            tmp_ids.cbegin(), tmp_ids.cend(), [this, &ball_pos, &ball_vel](auto a, auto b) {
              return (ball_pos - robot_pos_.at(a)).norm() /
                         (std::max(ball_vel.dot((robot_pos_.at(a) - ball_pos).normalized()),
                                   0.0) +
                          2000.0) <
                     (ball_pos - robot_pos_.at(b)).norm() /
                         (std::max(ball_vel.dot((robot_pos_.at(b) - ball_pos).normalized()),
                                   0.0) +
                          2000.0);
            });
      } else {
        chaser_ =
            *std::min_element(candidates.cbegin(), candidates.cend(), [this](auto a, auto b) {
              return (robot_pos_.at(a) - abp_target_).squaredNorm() <
                     (robot_pos_.at(b) - abp_target_).squaredNorm();
            });
      }
      const auto end = std::remove(tmp_ids.begin(), tmp_ids.end(), chaser_);
      tmp_ids.erase(end, tmp_ids.end());
    }

    // receiver /////////////////////////////
    if (is_active_ && !tmp_ids.empty() && pass_flag) {
      const auto receiver_itr =
          std::min_element(tmp_ids.cbegin(), tmp_ids.cend(), [this](auto a, auto b) {
            return (robot_pos_.at(a) - abp_target_).norm() <
                   (robot_pos_.at(b) - abp_target_).norm();
          });
      current_receiver_ = *receiver_itr;
      const auto end    = std::remove(tmp_ids.begin(), tmp_ids.end(), current_receiver_);
      tmp_ids.erase(end, tmp_ids.end());
      tmp_ids.shrink_to_fit();
    }

    // waiter ///////////////////////////////
    { waiters_ = tmp_ids; }
  }

  /////////////////////////////////////////
  // 動作を設定 ///////////////////////////
  {
    // chaser がボールを受け取ろうとしているか
    bool chaser_receiving = false;

    // chaser /////////////////////////////
    if (is_active_ && our_robots.count(chaser_)) {
      const auto id                    = chaser_;
      const Eigen::Vector2d& robot_pos = robot_pos_.at(id);
      // 待機位置からの許容距離
      constexpr double receiver_margin = 200.0;
      if (ball_vel.dot((robot_pos - ball_pos).normalized()) > 1000.0) {
        chaser_receiving = pass_flag;
        receive_.at(id)->set_dribble(5);
        baseaction.push_back(receive_.at(id));
      } else if (std::abs(ball_pos.x()) > wf.x_max() || std::abs(ball_pos.y()) > wf.y_max() ||
                 !pass_flag || !our_robots.count(current_receiver_) ||
                 (robot_pos_.at(current_receiver_) - abp_target_).norm() > receiver_margin) {
        abp_.at(id)->set_kick_type({model::command::kick_type_t::none, 0});
        baseaction.push_back(abp_.at(id));
      } else if (pass_flag &&
                 (robot_pos_.at(current_receiver_) - abp_target_).norm() < receiver_margin) {
        abp_.at(id)->set_kick_type({model::command::kick_type_t::line, 50});
        baseaction.push_back(abp_.at(id));
      } else {
        abp_.at(id)->set_kick_type({model::command::kick_type_t::none, 0});
        baseaction.push_back(abp_.at(id));
      }
    }

    // receiver & waiter ///////////////////////////
    planner::obstacle_list common_obstacles;
    common_obstacles.add(model::obstacle::enemy_penalty_area(wf, 150.0));
    common_obstacles.add(model::obstacle::our_penalty_area(wf, 150.0));
    for (const auto& robot : ene_robots) {
      common_obstacles.add(model::obstacle::point{util::math::position(robot.second), 200.0});
    }

    if (is_active_ && pass_flag) waiters_.push_back(current_receiver_);
    int count = 0;
    for (auto id : waiters_) {
      if (!our_robots.count(id)) continue;
      const bool is_receiver =
          is_active_ && id == current_receiver_ && pass_flag && !chaser_receiving;
      const Eigen::Vector2d& robot_pos = robot_pos_.at(id);
      const Eigen::Vector2d base_point(wf.x_min() + wf.penalty_length() + 1000.0, 0.0);
      // 待機位置の間隔
      constexpr double wait_diff = 350.0;
      const double theta =
          std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x());
      const Eigen::Vector2d target_pos =
          is_receiver
              ? static_cast<Eigen::Vector2d>(abp_target_ - 100.0 * (Eigen::Rotation2Dd(theta) *
                                                                    Eigen::Vector2d::UnitX()))
              : base_point + wait_diff * (count % 2 == 0 ? 1.0 : -1.0) *
                                 Eigen::Vector2d(0, count / 2 + count % 2);
      // 待機する
      auto hl = std::make_unique<planner::human_like>();
      hl->set_area(wf, 200.0);
      auto obstacles = common_obstacles;
      for (const auto& robot : our_robots) {
        if (robot.first != id)
          obstacles.add(model::obstacle::point{util::math::position(robot.second), 200.0});
      }

      if (!is_receiver) {
        obstacles.add(model::obstacle::segment{{abp_target_, ball_pos}, 650.0});
      }
      move_.at(id)->move_to(target_pos, theta);
      baseaction.push_back(
          std::make_shared<action::with_planner>(move_.at(id), std::move(hl), obstacles));
      count++;
    }
  }
  return baseaction;
}

} // namespace ai_server::game::agent
