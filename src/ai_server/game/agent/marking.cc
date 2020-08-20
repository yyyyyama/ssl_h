#include <cmath>
#include <algorithm>

#include "ai_server/game/action/get_ball.h"
#include "ai_server/game/action/marking.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/vec.h"
#include "ai_server/game/action/with_planner.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/human_like.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "marking.h"

using boost::math::constants::pi;

namespace ai_server {
namespace game {
namespace agent {

marking::marking(context& ctx, const std::vector<unsigned int>& ids, bool setplay_flag)
    : base(ctx), ids_(ids), setplay_flag_(setplay_flag) {
  const auto our_robots   = model::our_robots(world(), team_color());
  const auto enemy_robots = model::enemy_robots(world(), team_color());

  marker_ids_ = ids_;
  const auto m_end =
      std::remove_if(marker_ids_.begin(), marker_ids_.end(),
                     [&](const unsigned int x) { return our_robots.count(x) == 0; });
  marker_ids_.erase(m_end, marker_ids_.end());

  if (!our_robots.empty() && !enemy_robots.empty()) {
    for (const auto& enemy : enemy_robots) {
      if (enemy.second.x() < world().field().x_max() - 1600.0 ||
          std::abs(enemy.second.y()) > 1600.0) {
        // 敵ゴールエリア周辺以外の敵ロボットをマーク対象として設定
        enemy_ids_.push_back(enemy.first);
      }
    }

    if (!enemy_ids_.empty()) {
      // 敵ロボットを自チームゴールに近い順にソート
      std::sort(enemy_ids_.begin(), enemy_ids_.end(), [&](const auto& a, const auto& b) {
        return std::hypot(enemy_robots.at(a).x() - world().field().x_min(),
                          enemy_robots.at(a).y() - 0.0) <
               std::hypot(enemy_robots.at(b).x() - world().field().x_min(),
                          enemy_robots.at(b).y() - 0.0);
      });

      // マーク対象を自チームマーカー台数分抽出
      if (enemy_ids_.size() > marker_ids_.size()) {
        enemy_ids_.resize(marker_ids_.size());
      }

      auto marker_ids = marker_ids_;

      // 割り当て
      while (marker_ids.size() != 0) {
        for (const auto eid : enemy_ids_) {
          if (marker_ids.size() != 0) {
            const auto id = std::min_element(
                marker_ids.cbegin(), marker_ids.cend(),
                [&enemy_robots, &our_robots, &eid](const auto& a, const auto& b) {
                  return std::hypot(our_robots.at(a).x() - enemy_robots.at(eid).x(),
                                    our_robots.at(a).y() - enemy_robots.at(eid).y()) <
                         std::hypot(our_robots.at(b).x() - enemy_robots.at(eid).x(),
                                    our_robots.at(b).y() - enemy_robots.at(eid).y());
                });
            mark_pairs_.insert(std::make_pair(*id, eid));
            marker_ids.erase(id);
          }
        }
      }
    }
  }
}

void marking::set_setplay_flag(bool setplay_flag) {
  setplay_flag_ = setplay_flag;
}

std::vector<std::shared_ptr<action::base>> marking::execute() {
  std::vector<std::shared_ptr<action::base>> baseaction;
  const auto our_robots   = model::our_robots(world(), team_color());
  const auto enemy_robots = model::enemy_robots(world(), team_color());
  const auto ball         = world().ball();
  // 障害物としてのロボット半径
  constexpr double obs_robot_rad = 300.0;
  // 障害物としての敵ロボットの半径
  constexpr double obs_enemy_rad = 150.0;
  // フィールドから出られる距離
  constexpr double field_margin = 200.0;
  // ペナルティエリアから余裕を持たせる距離
  constexpr double penalty_margin = 150.0;
  // 一般障害物設定
  planner::obstacle_list common_obstacles;
  {
    for (const auto& robot : enemy_robots) {
      common_obstacles.add(
          model::obstacle::point{util::math::position(robot.second), obs_enemy_rad});
    }
    common_obstacles.add(model::obstacle::enemy_penalty_area(world().field(), penalty_margin));
    common_obstacles.add(model::obstacle::our_penalty_area(world().field(), penalty_margin));
  }

  using boost::math::constants::pi;

  marker_ids_ = ids_;
  const auto m_end =
      std::remove_if(marker_ids_.begin(), marker_ids_.end(),
                     [&](const unsigned int x) { return our_robots.count(x) == 0; });
  marker_ids_.erase(m_end, marker_ids_.end());

  if (marker_ids_.empty()) {
    // 自チームロボットが全く見えなければ何もせず終了
    return baseaction;
  }

  if (enemy_robots.empty()) {
    // 敵ロボットが全く見えない場合
    for (const auto id : marker_ids_) {
      const auto robot = our_robots.at(id);
      if (std::abs(robot.x()) > world().field().x_max() - 1300.0 &&
          std::abs(robot.y()) < 1300.0) {
        // ゴールエリアから出る
        double vx, vy;
        double goal_x = robot.x() > 0.0 ? world().field().x_max() : world().field().x_min();
        vx = 500.0 * (robot.x() - goal_x) / std::hypot(robot.x() - goal_x, robot.y() - 0.0);
        vy = 500.0 * (robot.y() - 0.0) / std::hypot(robot.x() - goal_x, robot.y() - 0.0);
        auto vec = make_action<action::vec>(id);
        vec->move_at(vx, vy, 0.0);
        baseaction.push_back(vec);
      } else {
        // 停止
        auto vec = make_action<action::vec>(id);
        vec->move_at(0.0, 0.0, 0.0);
        baseaction.push_back(vec);
      }
    }
    return baseaction;
  }

  std::vector<unsigned int> tmp_1 = enemy_ids_;
  enemy_ids_.clear();
  for (const auto& enemy : enemy_robots) {
    if (enemy.second.x() < world().field().x_max() - 1600.0 ||
        std::abs(enemy.second.y()) > 1600.0) {
      // 敵ゴールエリア周辺以外の敵ロボットをマーク対象として設定
      enemy_ids_.push_back(enemy.first);
    }
  }

  if (enemy_ids_.empty()) {
    // 敵マーク対象が存在しない場合
    for (const auto id : marker_ids_) {
      const auto robot = our_robots.at(id);
      if (std::abs(robot.x()) > world().field().x_max() - 1300.0 &&
          std::abs(robot.y()) < 1300.0) {
        // ゴールエリアから出る
        double vx, vy;
        double goal_x = robot.x() > 0.0 ? world().field().x_max() : world().field().x_min();
        vx = 500.0 * (robot.x() - goal_x) / std::hypot(robot.x() - goal_x, robot.y() - 0.0);
        vy = 500.0 * (robot.y() - 0.0) / std::hypot(robot.x() - goal_x, robot.y() - 0.0);
        auto vec = make_action<action::vec>(id);
        vec->move_at(vx, vy, 0.0);
        baseaction.push_back(vec);
      } else {
        // 停止
        auto vec = make_action<action::vec>(id);
        vec->move_at(0.0, 0.0, 0.0);
        baseaction.push_back(vec);
      }
    }
    return baseaction;
  }

  // 敵ロボットを自チームゴールに近い順にソート
  std::sort(enemy_ids_.begin(), enemy_ids_.end(), [&](const auto& a, const auto& b) {
    return std::hypot(enemy_robots.at(a).x() - world().field().x_min(),
                      enemy_robots.at(a).y() - 0.0) <
           std::hypot(enemy_robots.at(b).x() - world().field().x_min(),
                      enemy_robots.at(b).y() - 0.0);
  });

  // マーク対象を自チームマーカー台数分抽出
  if (enemy_ids_.size() > marker_ids_.size()) {
    enemy_ids_.resize(marker_ids_.size());
  }
  std::vector<unsigned int> tmp_2 = enemy_ids_;

  std::sort(tmp_1.begin(), tmp_1.end());
  std::sort(tmp_2.begin(), tmp_2.end());

  if (!std::equal(tmp_1.cbegin(), tmp_1.cend(), tmp_2.cbegin(), tmp_2.cend())) {
    // マーク対象のロボットに変更があれば再割り当て
    mark_pairs_.clear();

    auto marker_ids = marker_ids_;
    // 割り当て
    while (marker_ids.size() != 0) {
      for (const auto eid : enemy_ids_) {
        if (marker_ids.size() != 0) {
          const auto id = std::min_element(
              marker_ids.cbegin(), marker_ids.cend(),
              [&enemy_robots, &our_robots, &eid](const auto& a, const auto& b) {
                return std::hypot(our_robots.at(a).x() - enemy_robots.at(eid).x(),
                                  our_robots.at(a).y() - enemy_robots.at(eid).y()) <
                       std::hypot(our_robots.at(b).x() - enemy_robots.at(eid).x(),
                                  our_robots.at(b).y() - enemy_robots.at(eid).y());
              });
          mark_pairs_.insert(std::make_pair(*id, eid));
          marker_ids.erase(id);
        }
      }
    }
  }

  const auto marker_ids = marker_ids_;

  std::vector<unsigned int> our_ids;
  for (const auto& our : our_robots) {
    our_ids.push_back(our.first);
  }
  const auto nearest_id = std::min_element(
      our_ids.cbegin(), our_ids.cend(),
      [&our_robots, &ball](const unsigned int a, const unsigned int b) {
        return std::hypot(our_robots.at(a).x() - ball.x(), our_robots.at(a).y() - ball.y()) <
               std::hypot(our_robots.at(b).x() - ball.x(), our_robots.at(b).y() - ball.y());
      });
  const auto nid = *nearest_id;
  our_ids.erase(nearest_id);

  // それぞれのロボットに動作設定
  for (auto itr = mark_pairs_.cbegin(); itr != mark_pairs_.cend(); itr++) {
    const auto id     = itr->first;
    const auto eid    = itr->second;
    const auto& robot = our_robots.at(id);
    // 自チームロボットを障害物設定
    planner::obstacle_list obstacles = common_obstacles;
    for (const auto& robot : our_robots) {
      if (robot.first == id) continue;
      obstacles.add(model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    // planner::human_likeを使用
    auto hl = std::make_unique<planner::human_like>();
    hl->set_area(world().field(), field_margin);

    // 基本的にはkick_block、同一対象に複数台のマーカーがいる場合、最も近いロボット以外はshoot_block
    auto mark_mode =
        std::any_of(marker_ids_.cbegin(), marker_ids_.cend(),
                    [&](const unsigned int x) {
                      return mark_pairs_.at(x) == eid && x != id &&
                             std::hypot(robot.x() - world().field().x_min(), robot.y() - 0.0) <
                                 std::hypot(our_robots.at(x).x() - world().field().x_min(),
                                            our_robots.at(x).y() - 0.0);
                    })
            ? action::marking::mark_mode::shoot_block
            : action::marking::mark_mode::kick_block;

    if (our_robots.count(id) == 0) {
      // 停止
      auto vec = make_action<action::vec>(id);
      vec->move_at(0.0, 0.0, 0.0);
      baseaction.push_back(
          std::make_shared<action::with_planner>(vec, std::move(hl), obstacles));
    } else if (std::abs(robot.x()) > world().field().x_max() - 1300.0 &&
               std::abs(robot.y()) < 1300.0) {
      // ゴールエリアから出る
      double vx, vy;
      const double goal_x = robot.x() > 0.0 ? world().field().x_max() : world().field().x_min();
      vx       = 500.0 * (robot.x() - goal_x) / std::hypot(robot.x() - goal_x, robot.y() - 0.0);
      vy       = 500.0 * (robot.y() - 0.0) / std::hypot(robot.x() - goal_x, robot.y() - 0.0);
      auto vec = make_action<action::vec>(id);
      vec->move_at(vx, vy, 0.0);
      baseaction.push_back(
          std::make_shared<action::with_planner>(vec, std::move(hl), obstacles));
    } else if (setplay_flag_) {
      // 敵チームセットプレイ時
      if (id == nid && enemy_robots.count(eid)) {
        auto r_theta = util::math::wrap_to_2pi(std::atan2(ball.y() - enemy_robots.at(eid).y(),
                                                          ball.x() - enemy_robots.at(eid).x()));
        auto x       = ball.x() + 650 * std::cos(r_theta);
        auto y       = ball.y() + 650 * std::sin(r_theta);
        if (std::abs(x) > world().field().x_max() || std::abs(y) > world().field().y_max()) {
          const auto b2gtheta = util::math::wrap_to_2pi(
              std::atan2(0 - ball.y(), world().field().x_min() - ball.x()));
          x = ball.x() + 650 * std::sin(b2gtheta);
          y = ball.y() + 650 * std::cos(b2gtheta);
        }
        const auto to_ball_theta =
            util::math::wrap_to_2pi(std::atan2(ball.y() - y, ball.x() - x));
        auto move = make_action<action::move>(id);
        move->move_to(x, y, to_ball_theta);
        baseaction.push_back(
            std::make_shared<action::with_planner>(move, std::move(hl), obstacles));
      } else if (std::hypot(robot.x() - ball.x(), robot.y() - ball.y()) < 600.0) {
        // ボールから離れる
        double vx, vy;
        const double c_to_rtheta =
            util::math::wrap_to_2pi(std::atan2(robot.y() - 0.0, robot.x() - 0.0));
        const double c_to_btheta =
            util::math::wrap_to_2pi(std::atan2(ball.y() - 0.0, ball.x() - 0.0));

        if (std::abs(robot.x()) < world().field().x_max() - 750.0 &&
            std::abs(robot.y()) < world().field().y_max() - 750.0) {
          vx = (650.0 - std::hypot(robot.x() - ball.x(), robot.y() - ball.y())) *
               (robot.x() - ball.x()) / std::hypot(robot.x() - ball.x(), robot.y() - ball.y());
          vy = (650.0 - std::hypot(robot.x() - ball.x(), robot.y() - ball.y())) *
               (robot.y() - ball.y()) / std::hypot(robot.x() - ball.x(), robot.y() - ball.y());
        } else {
          vx = 1000.0 * std::sin(c_to_rtheta) *
               ((c_to_btheta > c_to_rtheta) - (c_to_btheta < c_to_rtheta));
          vy = -1000.0 * std::cos(c_to_rtheta) *
               ((c_to_btheta > c_to_rtheta) - (c_to_btheta < c_to_rtheta));
        }
        auto vec = make_action<action::vec>(id);
        vec->move_at(vx, vy, 0.0);
        baseaction.push_back(
            std::make_shared<action::with_planner>(vec, std::move(hl), obstacles));
      } else {
        if (enemy_robots.count(eid) != 0) {
          // 対象が見えればマーク
          auto mark = make_action<action::marking>(id);
          mark->mark_robot(eid);
          mark->set_mode(mark_mode);
          mark->set_radius(300.0);
          baseaction.push_back(
              std::make_shared<action::with_planner>(mark, std::move(hl), obstacles));
        } else {
          // 停止
          auto vec = make_action<action::vec>(id);
          vec->move_at(0.0, 0.0, 0.0);
          baseaction.push_back(
              std::make_shared<action::with_planner>(vec, std::move(hl), obstacles));
        }
      }
    } else {
      if ((std::abs(ball.x()) < world().field().x_max() - 1200.0 ||
           std::abs(ball.y()) > 1200.0) &&
          std::hypot(robot.x() - ball.x(), robot.y() - ball.y()) < 700.0 && id == nid &&
          std::all_of(our_ids.cbegin(), our_ids.cend(),
                      [&our_robots, &ball, &marker_ids](const unsigned int our_id) {
                        return std::any_of(marker_ids.cbegin(), marker_ids.cend(),
                                           [&our_id](const unsigned int mid) {
                                             return mid == our_id;
                                           }) ||
                               std::hypot(our_robots.at(our_id).x() - ball.x(),
                                          our_robots.at(our_id).y() - ball.y()) > 700.0;
                      })) {
        // ボールに近く、ボールの近くに味方ロボットがいない場合、ボールを相手ゴール方向に蹴る
        auto get_ball = make_action<action::get_ball>(id);
        get_ball->set_target(world().field().x_max(), 0.0);
        baseaction.push_back(
            std::make_shared<action::with_planner>(get_ball, std::move(hl), obstacles));
      } else {
        if (enemy_robots.count(eid) != 0) {
          // マーク
          if (std::any_of(
                  our_ids.cbegin(), our_ids.cend(),
                  [&our_robots, &enemy_robots, &marker_ids, &eid](const unsigned int our_id) {
                    return std::all_of(
                               marker_ids.cbegin(), marker_ids.cend(),
                               [&our_id](const unsigned int mid) { return mid != our_id; }) &&
                           std::hypot(enemy_robots.at(eid).x() - our_robots.at(our_id).x(),
                                      enemy_robots.at(eid).y() - our_robots.at(our_id).y()) <
                               500.0;
                  })) {
            // 近くにマーカー以外のロボットがいればモード変更
            mark_mode = action::marking::mark_mode::shoot_block;
          }
          auto mark = make_action<action::marking>(id);
          mark->mark_robot(eid);
          mark->set_mode(mark_mode);
          mark->set_radius(300.0);
          baseaction.push_back(
              std::make_shared<action::with_planner>(mark, std::move(hl), obstacles));
        } else {
          // 停止
          auto vec = make_action<action::vec>(id);
          vec->move_at(0.0, 0.0, 0.0);
          baseaction.push_back(
              std::make_shared<action::with_planner>(vec, std::move(hl), obstacles));
        }
      }
    }
  }
  return baseaction;
}
} // namespace agent
} // namespace game
} // namespace ai_server
