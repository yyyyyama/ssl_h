#include <algorithm>
#include <cmath>
#include "regular.h"

namespace ai_server {
namespace game {
namespace agent {

regular::regular(const model::world& world, bool is_yellow, const std::vector<unsigned int> ids)
    : base(world, is_yellow),
      ids_(ids),
      has_chaser_(true),
      chase_finished_(false),
      kick_finished_(false) {
  // markingとno_operationのアクションを生成
  for (auto id : ids_) {
    no_op_[id]   = std::make_shared<action::no_operation>(world_, is_yellow_, id);
    marking_[id] = std::make_shared<action::marking>(world_, is_yellow_, id);
  }
  update_formation();
}

bool regular::has_chaser() const {
  return has_chaser_;
}

void regular::use_chaser(bool chase_ball) {
  if (has_chaser_ != chase_ball) {
    has_chaser_ = chase_ball;
    update_formation();
  }
}

std::vector<std::shared_ptr<action::base>> regular::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();

  // ボールを追いかける
  if (has_chaser_) {
    // ボールから遠すぎるときは組みなおす
    const double chaser_ball = std::hypot(
        ids_robots.at(chaser_id_).x() - ball.x(),
        ids_robots.at(chaser_id_).y() - ball.y()); // 現在のchaserからボールまでの距離
    auto chase_id_tmp = select_chaser();
    if (chaser_ball > 900 &&
        chaser_ball - std::hypot(ids_robots.at(chase_id_tmp).x() - ball.x(),
                                 ids_robots.at(chase_id_tmp).y() - ball.y()) >
            1200) {
      update_formation();
    }

    if (chase_finished_) {
      if (kick_finished_) {
        // ReTry
        kick_finished_  = false;
        chase_finished_ = false;
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

  set_marking_normal();

  for (auto id_marking : marking_) {
    if (!has_chaser_ || id_marking.first != chaser_id_) {
      if (std::find(no_op_ids_.begin(), no_op_ids_.end(), id_marking.first) ==
          no_op_ids_.end()) {
        actions.push_back(id_marking.second); // markingを割り当て
      } else {
        actions.push_back(no_op_.at(id_marking.first)); // no_operationを割り当て
      }
    }
  }

  return actions;
}

// chase id_の候補を取得
unsigned int regular::select_chaser() {
  const auto ball = world_.ball();
  auto tmp_id_itr = nearest_id(ids_, ball.x(), ball.y());
  if (tmp_id_itr == ids_.end()) {
    return chaser_id_;
  } else {
    return *tmp_id_itr;
  }
}

// 全て組み直す
void regular::update_formation() {
  chase_finished_ = false;
  kick_finished_  = false;

  // ボールを追いかける担当のロボットを登録
  if (has_chaser_) {
    chaser_id_ = select_chaser();
    std::printf("chase_id:%d\n", chaser_id_);
    chase_ball_  = std::make_shared<action::chase_ball>(world_, is_yellow_, chaser_id_);
    kick_action_ = std::make_shared<action::kick_action>(world_, is_yellow_, chaser_id_);
  }

  set_marking_all();
}

// マーキングの設定(通常)
void regular::set_marking_normal() {
  bool set_all  = false;
  auto list_new = make_importance_list();
  auto list_old = importance_list_;

  // 敵の数or優先度順が変化したら一から再設定
  if (list_new.size() == list_old.size()) {
    while (!list_new.empty()) {
      if (list_new.top().id != list_old.top().id) {
        set_all = true;
        break;
      }
      list_new.pop();
      list_old.pop();
    }
  } else {
    set_all = true;
  }

  if (set_all) {
    set_marking_all();
  } else {
    make_markers(true);
  }
}

// マーキングの設定（全て再設定）
void regular::set_marking_all() {
  make_markers(false);
  make_markers(true);
}

// マーキング割り当て
void regular::make_markers(bool use_follower) {
  if (marking_.empty() || no_op_.empty()) {
    return;
  }

  importance_list_        = make_importance_list();
  const auto ball         = world_.ball();
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  std::vector<unsigned int> tmp_ids;
  auto tmp_list           = importance_list_;
  
  if(use_follower){
    tmp_ids=followers_ids_;
  }else{
    for(auto id: ids_){
      if(!has_chaser_ || id!=chaser_id_){
        tmp_ids.push_back(id);
      }
    }
  }
  

  while (!tmp_ids.empty()) {
    if (tmp_list.empty()) {
      break;
    } else {
      const double goal_robot_theta = std::atan2(
          0.0 - those_robots.at(tmp_list.top().id).y(),
          world_.field().x_min() -
              those_robots.at(tmp_list.top().id).x()); //自チーム側のゴール->敵ロボの角度
      const double ball_robot_theta =
          std::atan2(ball.y() - those_robots.at(tmp_list.top().id).y(),
                     ball.x() - those_robots.at(tmp_list.top().id).x()); //ボール->敵ロボの角度
      if (!use_follower ||
          std::abs(goal_robot_theta - ball_robot_theta) > std::atan2(1.0, 3.0)) {
        // use_follower=false || マーキングロボットが重ならない時
        auto tmp_id_itr = nearest_id(tmp_ids, those_robots.at(tmp_list.top().id).x(),
                                     those_robots.at(tmp_list.top().id).y());
        if (tmp_id_itr != tmp_ids.end()) {
          marking_.at(*tmp_id_itr)->mark_robot(tmp_list.top().id);
          if (use_follower) {
            // 余ったロボット
            marking_.at(*tmp_id_itr)->set_mode(action::marking::mark_mode::kick_block);
            marking_.at(*tmp_id_itr)->set_radius(400.0);
            std::printf("-%d marks -> %d\n", *tmp_id_itr, tmp_list.top().id);
          } else {
            // 常にマークにつくロボット
            marking_.at(*tmp_id_itr)->mark_robot(tmp_list.top().id);
            marking_.at(*tmp_id_itr)->set_mode(action::marking::mark_mode::shoot_block);
            marking_.at(*tmp_id_itr)->set_radius(600.0);
            std::printf("*%d marks -> %d\n", *tmp_id_itr, tmp_list.top().id);
          }
          tmp_ids.erase(tmp_id_itr); // 最後に要素を消去
        }
      }
    }
    tmp_list.pop();
  }

  if (use_follower) {
    no_op_ids_ = tmp_ids; // 最後まで余ったロボット
  } else {
    followers_ids_ = tmp_ids; // 余ったロボット
    no_op_ids_     = tmp_ids; // 整合性維持
  }
}

// 敵IDと重要度を設定、ソートした結果を返す
std::priority_queue<regular::id_importance_> regular::make_importance_list() {
  std::priority_queue<regular::id_importance_> tmp_list;
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  unsigned int id; // 敵ロボットのID

  for (auto that_rob : those_robots) {
    id = that_rob.first;
    if (those_robots.at(id).x() < world_.field().x_max() * 0.65) {
      double score = 0.0; // 敵のマーク重要度を得点として格納する
      score += std::hypot(world_.field().x_max() - world_.field().x_min(),
                          world_.field().y_max() - world_.field().y_min()) -
               std::hypot(those_robots.at(id).x() - world_.field().x_min(),
                          those_robots.at(id).y()); // 絶対位置による得点
      tmp_list.push({id, score});
    }
  }
  return tmp_list;
}

// ターゲットに最も近いロボットIDのイテレータを返す
std::vector<unsigned int>::const_iterator regular::nearest_id(
    const std::vector<unsigned int>& can_ids, double target_x, double target_y) const {
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  return std::min_element(can_ids.begin(), can_ids.end(), [&target_x, &target_y, ids_robots](
                                                              unsigned int a, unsigned int b) {
    return std::hypot(ids_robots.at(a).x() - target_x, ids_robots.at(a).y() - target_y) <
           std::hypot(ids_robots.at(b).x() - target_x, ids_robots.at(b).y() - target_y);
  });
}

bool regular::id_importance_::operator<(const id_importance_& next) const {
  return importance < next.importance;
}

} // agent
} // game
} // ai_server
