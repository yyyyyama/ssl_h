#include <algorithm>
#include <cmath>
#include "regular.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace agent {

regular::regular(const model::world& world, bool is_yellow, const std::vector<unsigned int> ids)
    : base(world, is_yellow),
      ids_(ids),
      is_chase_(true),
      chase_finished_(false),
      kick_finished_(false) {
  // markingとno_operationのアクションを生成
  for (auto id : ids_) {
    ids_no_op_[id] = std::make_shared<action::no_operation>(world_, is_yellow_, id);
    ids_marking_[id] = std::make_shared<action::marking>(world_, is_yellow_, id);
  }
  set_chase_id();
  reset_marking(chase_id_, is_chase_);
}

bool regular::ball_chase() const {
  return is_chase_;
}

void regular::set_ball_chase(bool ball_chase) {
  if (is_chase_ != ball_chase) {
    is_chase_ = ball_chase;
    if (is_chase_) {
      set_chase_id();
    }
    reset_marking(chase_id_, is_chase_);
  }
}

std::vector<std::shared_ptr<action::base>> regular::execute() {
  std::vector<std::shared_ptr<action::base>> actions;

  // ボールを追いかける
  if (is_chase_) {
    if (chase_finished_) {
      if (kick_finished_) {
        // ReTry
        set_chase_id();
        reset_marking(chase_id_, is_chase_);

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


  set_marking(false);

  for (auto id_marking : ids_marking_) {
    if (std::find(no_ids_.begin(), no_ids_.end(), id_marking.first) == no_ids_.end()) {
      if(id_marking.first!=chase_id_){
       actions.push_back(id_marking.second); // markingを割り当て
      }
    } else {
      actions.push_back(ids_no_op_.at(id_marking.first)); // no_operationを割り当て
    }
  }
  
  return actions;
}

// chase id をセットする
void regular::set_chase_id(){
  const auto ball = world_.ball();
  chase_id_       = *nearest_id_itr(ball.x(), ball.y(), ids_); // ボール追従ロボ登録
}

// マーキングをaction生成から組み直す
void regular::reset_marking(unsigned int& chase_ball_id, bool ball_chase) {
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  std::vector<unsigned int> unadded_ids; // まだ動作の割り当てられていない味方ロボットのID

  chase_finished_ = false;
  kick_finished_  = false;

  // ボールを追いかける担当のロボットを登録
  if (ball_chase) {
    chase_ball_  = std::make_shared<action::chase_ball>(world_, is_yellow_, chase_ball_id);
    kick_action_ = std::make_shared<action::kick_action>(world_, is_yellow_, chase_ball_id);
  }

  set_marking(true);
}

// マーキングの設定
void regular::set_marking(bool change_all) {
  if(!ids_marking_.empty()){
    const auto ball         = world_.ball();
    const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
    std::vector<unsigned int> unadded_ids; // まだ動作の割り当てられていない味方ロボットのID

    for(auto id : ids_){
      if(id!=chase_id_){
        unadded_ids.push_back(id);
      }
    }

    // change_allをtrueにする必要があるかを調べる
    if (!change_all) {
      auto list_new = importance_list_now();
      auto list_old = importance_list_;

      if (list_new.size() == list_old.size()) {
        while (!list_new.empty()) {
          if (list_new.top().id != list_old.top().id) {
            change_all = true;
            break;
          }
          list_new.pop();
          list_old.pop();
        }
      } else {
        change_all = true;
      }
    }

    importance_list_ = importance_list_now();
    auto tmp_list    = importance_list_;

    if (change_all) {
      // マーキングを担当するロボットを割り当て
      while (!tmp_list.empty()) {
        if (!unadded_ids.empty()) {
          auto tmp_id_itr = nearest_id_itr(those_robots.at(tmp_list.top().id).x(),
                                           those_robots.at(tmp_list.top().id).y(), unadded_ids);
          ids_marking_.at(*tmp_id_itr)->mark_robot(tmp_list.top().id);
          ids_marking_.at(*tmp_id_itr)->set_mode(action::marking::mark_mode::shoot_block);
          ids_marking_.at(*tmp_id_itr)->set_radius(500.0);
          unadded_ids.erase(tmp_id_itr); // 最後に要素を消去
        } else {
          break;
        }
        tmp_list.pop();
      }
      flow_ids_ = unadded_ids;
    }

    // 余ったロボットを割り当て
    no_ids_  = flow_ids_;
    tmp_list = importance_list_;

    while (!no_ids_.empty()) {
      if (!tmp_list.empty()) {
        if (std::abs(std::atan2(those_robots.at(tmp_list.top().id).y(),
                                those_robots.at(tmp_list.top().id).x() - world_.field().x_min()) -
                     std::atan2(those_robots.at(tmp_list.top().id).y() - ball.y(),
                                those_robots.at(tmp_list.top().id).x() - ball.x())) >
            std::atan2(2.0, 3.0)) {
              // マーキングロボットが重ならない時のみ2台目のマークをたてる
          auto tmp_id_itr = nearest_id_itr(those_robots.at(tmp_list.top().id).x(),
                                           those_robots.at(tmp_list.top().id).y(), no_ids_);
          ids_marking_.at(*tmp_id_itr)->mark_robot(tmp_list.top().id);
          ids_marking_.at(*tmp_id_itr)->set_mode(action::marking::mark_mode::kick_block);
          ids_marking_.at(*tmp_id_itr)->set_radius(300.0);
          no_ids_.erase(tmp_id_itr); // 最後に要素を消去
        }
      } else {
        break;
      }
      tmp_list.pop();
    }
  }
}

// 敵IDと重要度を設定、ソートした結果を返す
std::priority_queue<regular::id_importance_> regular::importance_list_now() {
  std::priority_queue<regular::id_importance_> tmp_list;
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  unsigned int id; // 敵ロボットのID
  double x;        // 敵ロボットのx
  double y;        // 敵ロボットのy

  for (auto that_rob : those_robots) {
    id = that_rob.first;
    x  = those_robots.at(id).x();
    y  = those_robots.at(id).y();
    if (x < world_.field().x_max() * 0.6) {
      tmp_list.push({id, std::hypot(world_.field().x_max() - world_.field().x_min(),
                                    world_.field().y_max() - world_.field().y_min()) -
                             std::hypot(x - world_.field().x_min(), y)});
    }
  }
  return tmp_list;
}

// ターゲットに最も近いロボットIDのイテレータを返す
std::vector<unsigned int>::const_iterator regular::nearest_id_itr(
    double target_x, double target_y, const std::vector<unsigned int>& can_ids) const {
  if (!can_ids.empty()) {
    
    const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
    return std::min_element(
        can_ids.begin(), can_ids.end(),
        [&target_x, &target_y, ids_robots](unsigned int a, unsigned int b) {
          return std::hypot(ids_robots.at(a).x() - target_x, ids_robots.at(a).y() - target_y) <
                 std::hypot(ids_robots.at(b).x() - target_x, ids_robots.at(b).y() - target_y);
        });
  }
}

} // agent
} // game
} // ai_server
