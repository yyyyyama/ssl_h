#include <algorithm>
#include <cmath>
#include <boost/math/constants/constants.hpp>

#include "ai_server/game/action/with_planner.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/human_like.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"

#include "all.h"

using boost::math::constants::pi;
using namespace std::chrono_literals;

namespace ai_server::game::agent {

all::all(context& ctx, const std::vector<unsigned int>& ids, unsigned int keeper_id)
    : base(ctx),
      ids_(ids),
      pass_target_(Eigen::Vector2d(world().field().x_max(), 0.0)),
      target_id_(0),
      evaluator_(nnabla()),
      keeper_id_(keeper_id),
      goal_keep_(make_action<action::goal_keep>(keeper_id)),
      keeper_get_ball_(make_action<action::get_ball>(
          keeper_id, Eigen::Vector2d(world().field().x_max(), 0.0))) {
  const auto now = std::chrono::steady_clock::now();
  const Eigen::Vector2d ene_goal_pos(world().field().x_max(), 0.0);
  for (auto id : ids_) {
    get_ball_[id]   = make_action<action::get_ball>(id, ene_goal_pos);
    receive_[id]    = make_action<action::receive>(id);
    move_[id]       = make_action<action::move>(id);
    vec_[id]        = make_action<action::vec>(id);
    mark_[id]       = make_action<action::marking>(id);
    guard_[id]      = make_action<action::guard>(id);
    lost_point_[id] = now;
  }
}

std::vector<std::shared_ptr<action::base>> all::execute() {
  std::vector<std::shared_ptr<action::base>> baseaction;
  const auto wf                  = world().field();
  const Eigen::Vector2d ball_pos = util::math::position(world().ball());
  const Eigen::Vector2d ball_vel = util::math::velocity(world().ball());
  const auto our_robots          = model::our_robots(world(), team_color());
  const auto ene_robots          = model::enemy_robots(world(), team_color());
  const Eigen::Vector2d our_goal_pos(wf.x_min(), 0.0);
  const Eigen::Vector2d ene_goal_pos(wf.x_max(), 0.0);
  if (ids_.empty() && !our_robots.count(keeper_id_)) return baseaction;

  // kick power(line)
  constexpr int line_pow = 40;
  // kick power(chip)
  constexpr int chip_pow = 255;
  // ロボット半径
  constexpr double robot_rad = 100.0;
  // 障害物としてのロボット半径
  constexpr double obs_robot_rad = 300.0;
  // フィールドから出られる距離
  constexpr double field_margin = 200.0;
  // ペナルティエリアから余裕を持たせる距離
  constexpr double penalty_margin = 150.0;
  // 一般障害物設定
  planner::obstacle_list common_obstacles;
  {
    for (const auto& robot : ene_robots) {
      common_obstacles.add(
          model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    common_obstacles.add(model::obstacle::enemy_penalty_area(world().field(), penalty_margin));
    common_obstacles.add(model::obstacle::our_penalty_area(world().field(), penalty_margin));
  }

  ///////////////////////////////////////////
  // lost判定 ///////////////////////////////
  // filterの補間に任せる?
  // ids_の中で見えていると判定するもの
  std::vector<unsigned int> visible_ids;
  const auto now = std::chrono::steady_clock::now();
  for (const auto& robot : our_robots)
    if (lost_point_.count(robot.first)) lost_point_.at(robot.first) = now;

  for (const auto [id, lost] : lost_point_) {
    if (now - lost <= 1s) {
      visible_ids.push_back(id);
      if (our_robots.count(id)) robot_pos_[id] = util::math::position(our_robots.at(id));
    } else {
      robot_pos_.erase(id);
    }
  }
  auto end = std::remove(visible_ids.begin(), visible_ids.end(), keeper_id_);
  visible_ids.erase(end, visible_ids.end());

  // guardしたいリスト
  std::vector<Eigen::Vector2d> guard_list;
  for (const auto& robot : ene_robots) {
    const Eigen::Vector2d ene_pos = util::math::position(robot.second);
    if (((ene_pos - our_goal_pos).norm() < 5000.0 || (ene_pos - ball_pos).norm() < 500.0) &&
        std::all_of(
            guard_list.cbegin(), guard_list.cend(),
            [&our_goal_pos,
             og_ene_theta = std::atan2(ene_pos.y() - our_goal_pos.y(),
                                       ene_pos.x() - our_goal_pos.x())](const auto& pos) {
              return std::abs(util::math::wrap_to_pi(
                         std::atan2(pos.y() - our_goal_pos.y(), pos.x() - our_goal_pos.x()) -
                         og_ene_theta)) > 0.1 * pi<double>();
            })) {
      guard_list.push_back(ene_pos);
    }
  }
  if ((ball_pos - our_goal_pos).norm() < 5000.0 &&
      std::all_of(
          guard_list.cbegin(), guard_list.cend(),
          [og_b_theta =
               std::atan2(ball_pos.y() - our_goal_pos.y(), ball_pos.x() - our_goal_pos.x()),
           &our_goal_pos](const auto& pos) {
            return std::abs(util::math::wrap_to_pi(
                       std::atan2(pos.y() - our_goal_pos.y(), pos.x() - our_goal_pos.x()) -
                       og_b_theta)) > 0.1 * pi<double>();
          })) {
    guard_list.push_back(ball_pos);
  }

  ///////////////////////////////////////////
  // 役割の更新 /////////////////////////////
  if (!visible_ids.empty()) {
    auto tmp_ids = visible_ids;
    // chaser /////////////////////////////
    {
      std::vector<unsigned int> candidates;
      if (std::abs(pass_target_.x()) < wf.x_max() - wf.penalty_length() ||
          std::abs(pass_target_.y()) > wf.penalty_width() / 2.0) {
        for (auto id : tmp_ids) {
          if ((robot_pos_.at(id) - pass_target_).norm() < 500.0 &&
              ball_vel.dot((robot_pos_.at(id) - ball_pos).normalized()) > 1000.0 &&
              std::abs(util::math::wrap_to_pi(
                  std::atan2(ball_vel.y(), ball_vel.x()) -
                  std::atan2(robot_pos_.at(id).y() - ball_pos.y(),
                             robot_pos_.at(id).x() - ball_pos.x()))) < 0.1 * pi<double>())
            candidates.push_back(id);
        }
      }
      if (candidates.empty()) {
        std::unordered_map<unsigned int, double> score_list;
        score_list.reserve(tmp_ids.size());
        for (auto id : tmp_ids) {
          const Eigen::Vector2d br = robot_pos_.at(id) - ball_pos;
          score_list[id] = br.norm() / (std::max(ball_vel.dot(br.normalized()), 0.0) + 2000.0);
        }
        chaser_ =
            std::min_element(score_list.cbegin(), score_list.cend(),
                             [](const auto& a, const auto& b) { return a.second < b.second; })
                ->first;
      } else {
        chaser_ =
            *std::min_element(candidates.cbegin(), candidates.cend(), [this](auto a, auto b) {
              return (robot_pos_.at(a) - pass_target_).squaredNorm() <
                     (robot_pos_.at(b) - pass_target_).squaredNorm();
            });
      }
      const auto end = std::remove_if(tmp_ids.begin(), tmp_ids.end(),
                                      [this](auto i) { return i == chaser_; });
      tmp_ids.erase(end, tmp_ids.end());
    }

    // waiter ///////////////////////////
    { waiters_ = tmp_ids; }
  }

  // chaser /////////////////////////////
  // 無効な chaser が入っている可能性があるため， visible_ids が空でない必要がある
  if (!visible_ids.empty() && our_robots.count(chaser_)) {
    // MCTSでpass_targetを決定
    const detail::mcts::state state{ball_pos, chaser_};
    detail::mcts::node root_node(state);
    evaluator_.execute(wf, our_robots, ene_robots, our_goal_pos, ene_goal_pos, root_node);
    const auto& child_nodes = root_node.child_nodes;
    target_id_              = chaser_;
    bool dribble_flag       = false;
    if (child_nodes.empty()) {
      pass_target_ = ene_goal_pos;
    } else {
      const auto& selected_node =
          *std::max_element(child_nodes.cbegin(), child_nodes.cend(),
                            [](const auto& a, const auto& b) { return a.n < b.n; });
      pass_target_ = selected_node.state.ball_pos;
      target_id_   = selected_node.state.chaser;
      dribble_flag = target_id_ == chaser_ && (ene_goal_pos - pass_target_).norm() > 500.0;
    }

    const unsigned int id = chaser_;
    const auto& robot_pos = robot_pos_.at(id);
    const Eigen::Vector2d kick_pos =
        robot_pos + 100.0 * (Eigen::Rotation2D<double>(our_robots.at(id).theta()) *
                             Eigen::Vector2d::UnitX());

    // 自チームロボットを障害物設定
    planner::obstacle_list obstacles = common_obstacles;
    for (const auto& obs_pos : robot_pos_) {
      if (obs_pos.first == id) continue;
      obstacles.add(model::obstacle::point{obs_pos.second, obs_robot_rad});
    }
    // planner::human_likeを使用
    std::unique_ptr<planner::human_like> hl = std::make_unique<planner::human_like>();
    hl->set_area(wf, field_margin);

    if (std::abs(ball_pos.x()) > wf.x_max() || std::abs(ball_pos.y()) > wf.y_max()) {
      vec_.at(id)->move_at(0.0, 0.0, 0.0);
      baseaction.push_back(vec_.at(id));
    } else if (ball_vel.dot((robot_pos - ball_pos).normalized()) > 1500.0 &&
               (std::abs(util::math::wrap_to_pi(
                    std::atan2(robot_pos.y() - ball_pos.y(), robot_pos.x() - ball_pos.x()) -
                    std::atan2(ball_vel.y(), ball_vel.x()))) < 0.2 * pi<double>() ||
                (ball_pos - robot_pos).norm() < 500.0)) {
      const bool redirect_flag =
          std::abs(util::math::wrap_to_pi(
              std::atan2(ball_vel.y(), ball_vel.x()) + pi<double>() -
              std::atan2(pass_target_.y() - kick_pos.y(), pass_target_.x() - kick_pos.x()))) <
          0.4 * pi<double>();
      const auto kick_type =
          redirect_flag ? model::command::kick_type_t::line : model::command::kick_type_t::none;
      if (redirect_flag) receive_.at(id)->set_shoot(pass_target_);
      receive_.at(id)->set_kick_type({kick_type, line_pow});
      receive_.at(id)->set_dribble(9);
      receive_.at(id)->set_avoid_penalty(true);
      baseaction.push_back(
          std::make_shared<action::with_planner>(receive_.at(id), std::move(hl), obstacles));
    } else if ((std::abs(robot_pos.y()) > wf.penalty_width() / 2.0 + 200.0 ||
                std::abs(robot_pos.x()) < wf.x_max() - wf.penalty_length() - 200.0) &&
               std::abs(ball_pos.y()) < wf.penalty_width() / 2.0 + 200.0 &&
               std::abs(ball_pos.x()) > wf.x_max() - wf.penalty_length() - 200.0) {
      vec_.at(id)->move_at(0.0, 0.0, 0.0);
      baseaction.push_back(vec_.at(id));
    } else {
      const auto kick_type =
          dribble_flag ? model::command::kick_type_t::none : model::command::kick_type_t::line;
      get_ball_.at(id)->kick_manually({kick_type, line_pow});
      get_ball_.at(id)->set_target(pass_target_);
      // ボールを打つ目標がゴール外ならばget_ballにチップ判定してもらう
      if (!dribble_flag && (pass_target_.x() < wf.x_max() - wf.penalty_length() ||
                            std::abs(pass_target_.y()) > wf.penalty_width() / 2.0))
        get_ball_.at(id)->kick_automatically(line_pow, chip_pow);
      baseaction.push_back(
          std::make_shared<action::with_planner>(get_ball_.at(id), std::move(hl), obstacles));
    }
  }

  // waiter ///////////////////////////
  if (!waiters_.empty()) {
    std::unordered_map<unsigned int, Eigen::Vector2d> wait_pos;
    std::vector<Eigen::Vector2d> tmp_poss;
    // defense
    for (const auto& g : guard_list)
      tmp_poss.push_back(0.5 * (g - our_goal_pos) + our_goal_pos);
    // all
    {
      const double x = std::clamp(
          (ball_pos - ene_goal_pos).norm() > 6000.0 ? ball_pos.x() + 3000.0
                                                    : ene_goal_pos.x() - 3000.0,
          wf.x_min() + wf.penalty_length() + 500.0, wf.x_max() - wf.penalty_length() - 500.0);
      tmp_poss.emplace_back(x, std::min(1500.0, wf.y_max() - robot_rad));
      tmp_poss.emplace_back(x, std::max(-1500.0, wf.y_min() + robot_rad));
    }
    // center_back
    {
      const double x = (wf.x_min() + wf.penalty_length()) / 2.0;
      tmp_poss.emplace_back(x, std::min(1000.0, wf.y_max() - robot_rad));
      tmp_poss.emplace_back(x, std::max(-1000.0, wf.y_min() + robot_rad));
    }
    // under_top
    tmp_poss.emplace_back(
        std::clamp(ball_pos.x() - 2000.0, wf.x_min() + wf.penalty_length() + 500.0,
                   wf.x_max() - wf.penalty_length() - 500.0),
        0.0);
    // volante
    tmp_poss.emplace_back(std::min(600.0, wf.x_max() - wf.penalty_length() - 500.0),
                          std::min(2500.0, wf.y_max()));
    tmp_poss.emplace_back(std::min(600.0, wf.x_max() - wf.penalty_length() - 500.0),
                          std::max(-2500.0, wf.y_min()));
    if (!tmp_poss.empty() && (ball_pos - our_goal_pos).norm() > 5000.0) {
      const auto itr = std::min_element(
          tmp_poss.cbegin(), tmp_poss.cend(),
          [&chaser_pos = robot_pos_.at(chaser_)](const auto& a, const auto& b) {
            return (chaser_pos - a).squaredNorm() < (chaser_pos - b).squaredNorm();
          });
      tmp_poss.erase(itr);
    }
    auto tmp_ids = waiters_;
    for (const auto& tmp : tmp_poss) {
      const auto itr =
          std::min_element(tmp_ids.cbegin(), tmp_ids.cend(), [this, &tmp](auto a, auto b) {
            return (robot_pos_.at(a) - tmp).squaredNorm() <
                   (robot_pos_.at(b) - tmp).squaredNorm();
          });
      if (itr == tmp_ids.cend()) break;
      wait_pos[*itr] = tmp;
      const auto end =
          std::remove_if(tmp_ids.begin(), tmp_ids.end(), [&itr](auto i) { return i == *itr; });
      tmp_ids.erase(end, tmp_ids.end());
    }

    // action設定
    for (auto id : waiters_) {
      if (!our_robots.count(id)) continue;
      const Eigen::Vector2d& robot_pos = robot_pos_.at(id);
      const Eigen::Vector2d& pos =
          wait_pos.count(id) ? (id == target_id_ ? pass_target_ : wait_pos.at(id)) : robot_pos;

      // 自チームロボットを障害物設定
      planner::obstacle_list obstacles = common_obstacles;
      for (const auto& obs_pos : robot_pos_) {
        if (obs_pos.first == id) continue;
        obstacles.add(model::obstacle::point{obs_pos.second, obs_robot_rad});
      }
      // planner::human_likeを使用
      std::unique_ptr<planner::human_like> hl = std::make_unique<planner::human_like>();
      hl->set_area(wf, field_margin);

      if (pos.x() < wf.x_min() + wf.penalty_length() + penalty_margin &&
          std::abs(pos.y()) < wf.penalty_width() / 2.0 + penalty_margin) {
        // 自陣ゴール前で防御
        guard_.at(id)->move_to(pos.x(), pos.y());
        baseaction.push_back(guard_.at(id));
      } else {
        // 待機する
        move_.at(id)->move_to(
            pos, std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x()));
        baseaction.push_back(
            std::make_shared<action::with_planner>(move_.at(id), std::move(hl), obstacles));
      }
    }
  }

  // keeper
  if (our_robots.count(keeper_id_)) {
    if (wf.x_min() + 2.0 * robot_rad < ball_pos.x() &&
        ball_pos.x() < wf.x_min() + wf.penalty_length() - robot_rad &&
        std::abs(ball_pos.y()) < std::max(wf.penalty_length() - robot_rad, 0.0) &&
        ball_vel.norm() < 500.0) {
      keeper_get_ball_->kick_manually({model::command::kick_type_t::chip, chip_pow});
      baseaction.push_back(keeper_get_ball_); //配列を返すためにキーパーを統合する
    } else {
      baseaction.push_back(goal_keep_); //配列を返すためにキーパーを統合する
    }
  }
  return baseaction;
}
} // namespace ai_server::game::agent
