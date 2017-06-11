#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include "regular.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace agent {

regular::regular(const model::world& world, bool is_yellow,
                 const std::vector<unsigned int> ids)
    : base(world, is_yellow), ids_(ids),is_chase_(true),chase_finished_(false),kick_finished_(false) {
  const auto ball = world_.ball();
  chase_id_  = *nearest_id_itr(ball.x(), ball.y(), ids_); //ボール追従ロボ登録
  set_marking(chase_id_, is_chase_);
}

bool regular::ball_chase() const {
  return is_chase_;
}

void regular::set_ball_chase(bool ball_chase) {
  if (is_chase_ != ball_chase) {
    is_chase_ = ball_chase;

    if (is_chase_) {
      chase_finished_ = false;
      kick_finished_   = false;
      const auto ball = world_.ball();
      chase_id_ = *nearest_id_itr(ball.x(), ball.y(), ids_); // ボール追従ロボ登録
    }

    set_marking(chase_id_, is_chase_);
  }
}

std::vector<std::shared_ptr<action::base>> regular::execute() {
  std::vector<std::shared_ptr<action::base>> actions;

  // ボールを追いかける
  if (is_chase_) {
    if (chase_finished_) {
      if (kick_finished_) {
        // ReTry
        const auto ball                 = world_.ball();
        unsigned int tmp_chase_id = *nearest_id_itr(ball.x(), ball.y(), ids_);

        chase_id_  = tmp_chase_id;
        chase_finished_ = false;
        kick_finished_   = false;
        set_marking(chase_id_, is_chase_);

      } else {
        // Kick
        kick_action_->kick_to(world_.field().x_max(), 0.0);
        kick_action_->set_kick_type({model::command::kick_type_t::line, 50.0});
        kick_action_->set_mode(action::kick_action::mode::goal);
        kick_finished_ = kick_action_->finished();
        actions.push_back(kick_action_);
      }
    } else {
      // Cahse Ball
      chase_finished_ = chase_ball_->finished();
      actions.push_back(chase_ball_);
    }
  }

  // marking
  actions.insert(actions.end(), mark_actions_.begin(), mark_actions_.end());

  return actions;
}

void regular::set_marking(unsigned int& chase_ball_id, bool ball_chase) {
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  std::vector<unsigned int> unadded_ids;  // まだ動作の割り当てられていない味方ロボットのID
  std::vector<unsigned int> those_ids; // 敵ロボットのID

  mark_actions_.erase(mark_actions_.begin(), mark_actions_.end()); //リセット

  // 敵ロボットのIDを登録
  for (auto that_rob : those_robots) {
    those_ids.push_back(that_rob.first);
  }

  // マーキング担当のロボットを登録
  if (ball_chase) {
    std::printf("ChaseBall:%d\n",chase_id_);
    chase_ball_  = std::make_shared<action::chase_ball>(world_, is_yellow_, chase_ball_id);
    kick_action_ = std::make_shared<action::kick_action>(world_, is_yellow_, chase_ball_id);
  }

  for (auto id : ids_) {
    if (!(ball_chase && id == chase_id_)) {
      unadded_ids.push_back(id);
    }
  }

  // 対象敵のIDと重要度を登録し、重要度の高い順にソート
  std::priority_queue<regular::id_importance_> importance_list;
  for (auto id : those_ids) {
    importance_list.push(
        {id, -1.0 * std::hypot(those_robots.at(id).x() - world_.field().x_min(),
                               those_robots.at(id).y())});
  }

  // マーキングを担当するロボットを割り当て
  while(!importance_list.empty()) {
    if (!unadded_ids.empty()) {
      auto tmp_id_itr= nearest_id_itr(those_robots.at(importance_list.top().id).x(),those_robots.at(importance_list.top().id).y(), unadded_ids);
      marking_ = std::make_shared<action::marking>(world_, is_yellow_, *tmp_id_itr);
      marking_->mark_robot(importance_list.top().id);
      marking_->set_mode(action::marking::mark_mode::shoot_block);
      marking_->set_radius(300.0);
      mark_actions_.push_back(marking_);
      std::printf("%d --> %d  (importance: %f)\n",*tmp_id_itr,importance_list.top().id,importance_list.top().importance);
      unadded_ids.erase(tmp_id_itr); // 最後に要素を消去
    } else {
      break;
    }
    importance_list.pop();
  }
}


std::vector<unsigned int>::const_iterator regular::nearest_id_itr(double target_x, double target_y,
                                      const std::vector<unsigned int>& can_ids) const {
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if(!can_ids.empty()){
  return std::min_element(can_ids.begin(), can_ids.end(),[&target_x, &target_y, ids_robots](unsigned int a, unsigned int b) {
        return std::hypot(ids_robots.at(a).x() - target_x, ids_robots.at(a).y() - target_y) <
               std::hypot(ids_robots.at(b).x() - target_x, ids_robots.at(b).y() - target_y);
      });
  }
}

} // agent
} // game
} // ai_server
