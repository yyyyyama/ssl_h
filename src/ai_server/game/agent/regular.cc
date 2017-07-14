#include <algorithm>
#include <cmath>
#include "regular.h"
#include "ai_server/util/algorithm.h"

namespace ai_server {
namespace game {
namespace agent {

using ai_server::util::pop_each;

regular::regular(const model::world& world, bool is_yellow,
                 const std::vector<unsigned int>& ids)
    : base(world, is_yellow),
      ids_(ids),
      chase_finished_(false),
      kick_finished_(false),
      has_chaser_(true),
      need_update_(false) {
  set_chaser(select_chaser()); // Chaser の初期化
  update_all_marking();        // マーキングの初期化
}

bool regular::has_chaser() const {
  return has_chaser_;
}

void regular::use_chaser(bool use_chaser) {
  if (has_chaser_ != use_chaser) {
    has_chaser_  = use_chaser;
    need_update_ = true; // マーキング全体を再構成

    if (use_chaser) {
      set_chaser(select_chaser()); // chaserの設定
    } else {
      // Chaserをリセット
      chase_ball_  = nullptr;
      kick_action_ = nullptr;
    }
  }
}

std::vector<std::shared_ptr<action::base>> regular::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();

  if (has_chaser_) {
    // chaserを使う時

    const double chaser_ball = std::hypot(ids_robots.at(chaser_id_).x() - ball.x(),
                                          ids_robots.at(chaser_id_).y() - ball.y());
    const auto chaser_id_tmp = select_chaser();
    // chserがボールに追いつけない && 他のロボットの方がボールに対して明らかに近い時は交代
    if (chaser_ball > 800 &&
        chaser_ball - std::hypot(ids_robots.at(chaser_id_tmp).x() - ball.x(),
                                 ids_robots.at(chaser_id_tmp).y() - ball.y()) >
            1000) {
      regular::set_chaser(chaser_id_tmp); // chaserを再設定
      need_update_ = true;                // マーキングの再構成
    }

    // chaserへのaction割り当て
    if (chase_finished_) {
      if (kick_finished_) {
        // 蹴り終わったらリセット
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

  if (!need_update_) {
    // === 敵リストの変化状況を見て、必要ならマーキングを再構成

    auto list_old = importance_list_;       // 古いほう
    auto list_new = make_importance_list(); // 新しいほう

    // マークされていない敵の重要度が高くなり、マークする必要がでたときは再構成してマークする

    // ==詳細な条件==
    // 1 新しく取得したマーク対象の数が味方より多い
    // 2 新しく取得したマーク対象の数と味方の数が等しい
    // 3 新しく取得したマーク対象の数が味方より少ない
    // --判定--
    // 1 -> 1 重要度順が[味方の数]番目まで同じならset_allしない
    // 1 -> 2 重要度順が[味方の数]番目まで同じならset_allしない
    // 1 -> 3 set_allする
    // 2 -> 1 重要度順が[味方の数]番目まで同じならset_allしない
    // 2 -> 2 set_allしない
    // 2 -> 3 set_allする
    // 3 -> 1 set_allする
    // 3 -> 2 set_allする
    // 3 -> 3 敵の数が変化している時のみset_allする

    if ((has_chaser_ ? ids_.size() - 1 : ids_.size()) <= list_new.size()) {
      // マーキングロボの数がマーク対象の数以下の時（条件：1->1,2->1,3->1, 1->2,2->2,3->2）
      if (list_new.size() != list_old.size() ||
          (has_chaser_ ? ids_.size() - 1 : ids_.size()) != list_new.size()) {
        //マーキング対象の数＝味方ロボットの数 の状態が続いていない時(条件：1->3,3->3, 1->2,
        // 3->2)

        // 敵リストの順番が何処まで同じか調べる
        unsigned int c = 0;
        while (!list_new.empty() && !list_old.empty()) {
          if (list_new.top().id != list_old.top().id || c >= ids_.size()) {
            break;
          }
          list_new.pop();
          list_old.pop();
          c++;
        }
        if (c < (has_chaser_ ? ids_.size() - 1 : ids_.size())) {
          // 敵リストの順番が味方数の所まで同じではない時
          // (条件3->1,3->2もここに入る)
          need_update_ = true;
        }
      }
    } else {
      // マーキングロボの数が、マーク対象の数より多いとき(条件：3->3,2->3,1->3)
      if (list_new.size() != list_old.size()) {
        // 敵の数が変化している時
        need_update_ = true;
      }
    }
  }

  importance_list_ = make_importance_list(); // 敵リストの更新

  if (need_update_) {
    update_all_marking(); // マーキング全体を再構成
  } else {
    update_second_marking(); // 2枚目のマーキングのみ再構成
  }

  // 1枚目のマーキング
  if (!first_marking_.empty()) {
    for (auto&& mark : first_marking_) {
      actions.push_back(mark.second);
    }
  }

  // 2枚目のマーキング
  if (!second_marking_.empty()) {
    for (auto&& mark : second_marking_) {
      actions.push_back(mark.second);
    }
  }

  // 余ったロボット
  if (!no_op_.empty()) {
    for (auto&& no : no_op_) {
      actions.push_back(no.second);
    }
  }

  return actions;
}

// chase id_の候補を取得
unsigned int regular::select_chaser() {
  const auto ball = world_.ball();
  // ボールから最も近いロボットを選ぶ
  auto tmp_id_itr = nearest_id(ids_, ball.x(), ball.y());
  if (tmp_id_itr == ids_.end()) {
    return chaser_id_; // エラーの時は既に設定されている値を返す
  } else {
    return *tmp_id_itr;
  }
}

// chaserを設定し、chaserに必要なアクションを割り当てる
void regular::set_chaser(const unsigned int new_chaser_id) {
  if (!has_chaser_) {
    return; // use_chaserがfalseのときは動作しない
  }

  // 終了フラグをリセット
  chase_finished_ = false;
  kick_finished_  = false;

  // chaser_id_の設定
  chaser_id_ = new_chaser_id;

  // Actionの設定
  chase_ball_  = std::make_shared<action::chase_ball>(world_, is_yellow_, chaser_id_);
  kick_action_ = std::make_shared<action::kick_action>(world_, is_yellow_, chaser_id_);
}

// マーキングの設定（全て）
void regular::update_all_marking() {
  // 初期化
  follower_ids_.clear();
  first_marking_.clear();

  // chacer_は省く
  for (auto&& id : ids_) {
    if (!has_chaser_ || id != chaser_id_) {
      follower_ids_.push_back(id);
    }
  }

  // === 1枚目のマーキング(シュートブロック)

  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  auto tmp_list           = importance_list_;

  // 全ての味方ロボットが割り当てられるか、敵を全てマークするまで続ける
  while (!follower_ids_.empty() && !tmp_list.empty()) {
    // 対象から最も近いロボットに割り当てる
    const auto tmp_id_itr = nearest_id(follower_ids_, those_robots.at(tmp_list.top().id).x(),
                                       those_robots.at(tmp_list.top().id).y());
    if (tmp_id_itr != follower_ids_.end()) {
      first_marking_[*tmp_id_itr] =
          std::make_shared<action::marking>(world_, is_yellow_, *tmp_id_itr);
      first_marking_.at(*tmp_id_itr)->mark_robot(tmp_list.top().id);
      first_marking_.at(*tmp_id_itr)->set_mode(action::marking::mark_mode::shoot_block);
      first_marking_.at(*tmp_id_itr)->set_radius(400.0);
      follower_ids_.erase(tmp_id_itr); // 割り振ったら消す
    }
    tmp_list.pop();
  }

  // 2枚目のマーキング
  update_second_marking();

  need_update_ = false; // 再構成フラグをリセット
}

// マーキングの再構成(2枚目のマーキングのみ)
void regular::update_second_marking() {
  // リセット
  no_op_.clear();
  second_marking_.clear();

  // 余りがいなかったら終了
  if (follower_ids_.empty()) {
    return;
  }

  const auto ball         = world_.ball();
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  auto tmp_list           = importance_list_;
  auto tmp_ids            = follower_ids_;

  // 全ての味方ロボットが割り当てられるか、敵を全てマークするまで続ける
  while (!tmp_ids.empty() && !tmp_list.empty()) {
    const double goal_robot_theta = std::atan2(
        0.0 - those_robots.at(tmp_list.top().id).y(),
        world_.field().x_min() -
            those_robots.at(tmp_list.top().id).x()); //自チーム側のゴール->敵ロボの角度
    const double ball_robot_theta =
        std::atan2(ball.y() - those_robots.at(tmp_list.top().id).y(),
                   ball.x() - those_robots.at(tmp_list.top().id).x()); //ボール->敵ロボの角度

    if (std::abs(goal_robot_theta - ball_robot_theta) > std::atan2(1.0, 3.0)) {
      // マーキングが重ならないように
      const auto tmp_id_itr = nearest_id(tmp_ids, those_robots.at(tmp_list.top().id).x(),
                                         those_robots.at(tmp_list.top().id).y());
      if (tmp_id_itr != tmp_ids.end()) {
        // マーキングの設定
        second_marking_[*tmp_id_itr] =
            std::make_shared<action::marking>(world_, is_yellow_, *tmp_id_itr);
        second_marking_.at(*tmp_id_itr)->mark_robot(tmp_list.top().id);
        second_marking_.at(*tmp_id_itr)->set_mode(action::marking::mark_mode::kick_block);
        second_marking_.at(*tmp_id_itr)->set_radius(400.0);
        tmp_ids.erase(tmp_id_itr); // 割り振ったら消す
      }
    }
    tmp_list.pop();
  }

  // 余ったロボットにはno_op_を割り当てる
  if (!tmp_ids.empty()) {
    for (auto&& id : tmp_ids) {
      no_op_[id] = std::make_shared<action::no_operation>(world_, is_yellow_, id);
    }
  }
}

// 敵リストの作成
std::priority_queue<regular::id_importance> regular::make_importance_list() {
  std::priority_queue<regular::id_importance> tmp_list; // 一時的なリスト
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue(); // 味方
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue(); // 敵
  const auto ball         = world_.ball(); // ボール

  for (auto&& that_rob : those_robots) {
    const auto id         = that_rob.first;       // 敵ロボットのID
    const double that_x   = that_rob.second.x();  // 敵ロボのx座標
    const double that_y   = that_rob.second.y();  // 敵ロボのy座標
    const double robot_vx = that_rob.second.vx(); // ロボットのx速度

    // ゴールとの距離
    const double gall_dist = std::hypot(that_x - world_.field().x_min(), that_y);
    // ボールとの距離
    const double ball_dist = std::hypot(that_x - ball.x(), that_y - ball.y());
    // ゴールからの角度
    const double gall_angle = std::atan2(that_y, that_x - world_.field().x_min());
    // ゴールからみたボールの角度
    const double ball_gall_angle = std::atan2(ball.y(), ball.x() - world_.field().x_min());

    // ボールを持っているか
    bool is_kicker = ball_dist < 1000;

    // has_chaserにマークを委託
    if (has_chaser_ && is_kicker) {
      continue;
    }

    // 対象となるエリアの中か
    bool is_marking_area =
        // 敵側に寄りすぎていないか
        that_x < world_.field().x_max() * 0.65 &&
        // ペナルティエリア付近にいないか
        gall_dist > 2000 &&
        // ディフェンスの近くにいないか
        !(std::abs(ball_gall_angle - gall_angle) < atan2(1.2, 4.0) && gall_dist < 4000);

    // マーキングエリア内 && chaserとかぶらない時
    if (is_marking_area) {
      // 敵のマーク重要度
      double score = 0.0;
      // 距離の正規化用に使う
      const double gall_dist_max =
          std::hypot(world_.field().x_max() - world_.field().x_min(), world_.field().y_max());

      // ゴールに近い or ゴールに対して正面なほど優先度高め
      score += std::cos(gall_angle * 0.3) * (gall_dist_max - gall_dist) / gall_dist_max; // 位置

      // ある一定以上のスピードで移動している時
      if (std::abs(robot_vx) > 1100.0) {
        // こっち側に向かってくる程優先度高め
        // 爆走してきた時に優先度TOPになるように
        // -robot_vx/<爆走した時の速度>*(<重要度を覆したい分のx幅>/gall_dist_max）
        score -= robot_vx / 5000 * 0.8 * (world_.field().x_max() - world_.field().x_min()) /
                 gall_dist_max;
      }

      tmp_list.push({id, score});
    }
  }

  // マーキングのロボットが密集しないように、密集の元になる敵はリストから除外
  std::vector<unsigned int> added_list;
  std::priority_queue<regular::id_importance> list;

  pop_each(tmp_list, [&those_robots, &list, &added_list, this](auto&& item) {
    const auto id       = item.id;                 // 敵ロボのID
    const double that_x = those_robots.at(id).x(); // 敵ロボのx座標
    const double that_y = those_robots.at(id).y(); //敵ロボのy座標

    if (added_list.empty()) {
      // 初めて選ばれた要素のとき
      list.push(item);
      added_list.push_back(id);
    } else {
      // 最も角度差が小さいID
      auto next_id = most_overlapped_id(added_list, those_robots, that_x, that_y);
      if (next_id != added_list.end()) {
        const auto next_x = those_robots.at(*next_id).x();
        const auto next_y = those_robots.at(*next_id).y();
        // 最も角度が近いロボットとの角度差
        const auto smallest_theta =
            std::abs(std::atan2(that_y, that_x - world_.field().x_min()) -
                     std::atan2(next_y, next_x - world_.field().x_min()));

        if (smallest_theta > std::atan2(1.0, 3.0)) {
          // ある程度の角度差が保たれている時
          list.push(item);
          added_list.push_back(id);
        }
      }
    }
  });
  return list;
}

// ターゲットに最も近いロボットID
std::vector<unsigned int>::const_iterator regular::nearest_id(
    const std::vector<unsigned int>& can_ids, double target_x, double target_y) const {
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  return std::min_element(can_ids.begin(), can_ids.end(), [&target_x, &target_y, ids_robots](
                                                              unsigned int a, unsigned int b) {
    return std::hypot(ids_robots.at(a).x() - target_x, ids_robots.at(a).y() - target_y) <
           std::hypot(ids_robots.at(b).x() - target_x, ids_robots.at(b).y() - target_y);
  });
}

// ゴールからからみたとき、指定位置と最も角度の近い敵ロボットID
std::vector<unsigned int>::const_iterator regular::most_overlapped_id(
    std::vector<unsigned int>& those_ids,
    const std::unordered_map<unsigned int, model::robot>& those_robots, double target_x,
    double target_y) const {
  const double gall_x = world_.field().x_min();                        // ゴールのx座標
  const double target_theta = std::atan2(target_y, target_x - gall_x); // ゴールと指定地点の角度

  return std::min_element(those_ids.begin(), those_ids.end(), [&gall_x, &target_theta,
                                                               those_robots](unsigned int a,
                                                                             unsigned int b) {
    // ロボットaの角度差
    const double theta_a = std::atan2(those_robots.at(a).y(), those_robots.at(a).x() - gall_x);
    // ロボットbの角度差
    const double theta_b = std::atan2(those_robots.at(b).y(), those_robots.at(b).x() - gall_x);
    return std::abs(theta_a - target_theta) < std::abs(theta_b - target_theta);
  });
}

bool regular::id_importance::operator<(const id_importance& next) const {
  return importance < next.importance; // 重要度で比較
}

} // agent
} // game
} // ai_server
