#include <algorithm>
#include <cmath>

#include "ai_server/game/action/with_planner.h"
#include "ai_server/model/obstacle/field.h"
#include "ai_server/planner/human_like.h"
#include "ai_server/util/algorithm.h"
#include "ai_server/util/math/to_vector.h"

#include "regular.h"

namespace ai_server::game::agent {

using ai_server::util::pop_each;

regular::regular(context& ctx, const std::vector<unsigned int>& ids)
    : base(ctx),
      ids_(ids),
      can_chase_(true),
      has_chaser_(true),
      need_reset_chaser_(true),
      mark_option_(mark_option::normal) {
  chaser_id_ = select_chaser();
}

regular::regular(context& ctx, const std::vector<unsigned int>& ids,
                 unsigned int default_chaser)
    : base(ctx),
      ids_(ids),
      can_chase_(true),
      has_chaser_(true),
      need_reset_chaser_(true),
      mark_option_(mark_option::normal),
      chaser_id_(default_chaser) {}

bool regular::has_chaser() const {
  return has_chaser_;
}

unsigned int regular::chaser_id() const {
  return chaser_id_;
}

void regular::use_chaser(bool use_chaser) {
  has_chaser_ = use_chaser;

  if (use_chaser) {
    // chaserを初期化させる
    chaser_id_         = select_chaser();
    need_reset_chaser_ = true;
  } else {
    get_ball_ = nullptr;
  }
}

void regular::customize_marking(regular::mark_option option) {
  mark_option_ = option;
}

std::vector<std::shared_ptr<action::base>> regular::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
  const auto our_team       = model::our_robots(world(), team_color());
  const auto ball           = world().ball();
  const auto penalty_width  = world().field().penalty_width();
  const auto penalty_length = world().field().penalty_length();
  const double margin       = 200;
  // 障害物としてのロボット半径
  constexpr double obs_robot_rad = 300.0;
  // フィールドから出られる距離
  constexpr double field_margin = 200.0;
  // ペナルティエリアから余裕を持たせる距離
  constexpr double penalty_margin = 150.0;
  // 一般障害物設定
  planner::obstacle_list common_obstacles;
  {
    for (const auto& robot : model::enemy_robots(world(), team_color())) {
      common_obstacles.add(
          model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    common_obstacles.add(model::obstacle::enemy_penalty_area(world().field(), penalty_margin));
    common_obstacles.add(model::obstacle::our_penalty_area(world().field(), penalty_margin));
  }

  // chaserを使う時
  if (has_chaser_) {
    // 自分側のゴール
    regular::area my_goal{world().field().x_min(), -penalty_width / 2 - margin,
                          world().field().x_min() + penalty_length + margin,
                          penalty_width / 2 + margin, 0.0};
    // 相手側のゴール
    regular::area enemy_goal{world().field().x_max(), -penalty_width / 2 - margin,
                             world().field().x_max() - penalty_length - margin,
                             penalty_width / 2 + margin, 0.0};

    if ( // 自分側のゴール中心に入っているか
        regular::in_area(ball.x(), ball.y(), my_goal) ||
        // 相手側のゴール中心に入っているか
        regular::in_area(ball.x(), ball.y(), enemy_goal)) {
      // ボールを追いかけないようにする
      can_chase_ = false;
      get_ball_  = nullptr;

    } else if (
        //　前の段階でボールを追いかけなかったとき
        !can_chase_) {
      //　chaserを初期化させる
      can_chase_         = true;
      chaser_id_         = select_chaser();
      need_reset_chaser_ = true;
    }

    if (can_chase_) {
      // パスが受け取られたか
      bool passed = false;

      // パスが受け取れたかを調べる
      for (const auto& r : receive_) {
        passed = r.second->finished();
        if (passed) {
          // 受け取ったロボットをchaserにする
          chaser_id_         = r.first;
          need_reset_chaser_ = true;
          break;
        }
      }

      // パスが受け取られていないとき
      if (!passed) {
        // chaserが見えない時
        if (our_team.find(chaser_id_) == our_team.end()) {
          // 新しいchaserを設定
          chaser_id_         = select_chaser();
          need_reset_chaser_ = true;
        } else {
          const double chaser_ball = std::hypot(our_team.at(chaser_id_).x() - ball.x(),
                                                our_team.at(chaser_id_).y() - ball.y());
          // 現状で、ボールを取りに行くのにふさわしいロボット
          const auto tmp_chaser = select_chaser();

          if ( //　chserがボールに追いつけないか
              chaser_ball > 2500 &&
              //  他のロボットの方がボールに対して明らかに近いか
              chaser_ball - std::hypot(our_team.at(tmp_chaser).x() - ball.x(),
                                       our_team.at(tmp_chaser).y() - ball.y()) >
                  1300) {
            // chaserを交代
            chaser_id_         = tmp_chaser;
            need_reset_chaser_ = true;
          }
        }
      }

      // 受け取り手がいない時
      if (receive_.empty()) {
        // 蹴る先
        auto new_target = select_target();
        // 蹴る先-＞ボールの角度
        double target_angle = std::atan2(new_target.y() - ball.y(), new_target.x() - ball.x());
        // 前回の蹴る先-＞ボールの角度
        double old_target_angle = std::atan2(target_.y() - ball.y(), target_.x() - ball.x());

        // == 蹴る先は状況によって変わるので、変化が小さいときは弾く

        // 位置ずれの許容値を求める
        // tan(theta) = h/r
        // h = 蹴る先の地点からの許容誤差[mm]
        // r = ボール->蹴る先の距離
        double r       = std::hypot(target_.x() - ball.x(), target_.y() - ball.y());
        double max_deg = std::atan2(800.0, r);

        if (std::abs(old_target_angle - target_angle) > max_deg) {
          target_ = new_target;
        }
      }

      // chaserが見えるとき
      if (our_team.find(chaser_id_) != our_team.end()) {
        // 初期化が必要なとき
        if (need_reset_chaser_) {
          get_ball_          = make_action<action::get_ball>(chaser_id_);
          need_reset_chaser_ = false;
        }
        get_ball_->set_target(target_.x(), target_.y());
        get_ball_->kick_automatically((target_.x() == world().field().x_max() ? 110 : 60));
        planner::obstacle_list obstacles = common_obstacles;
        for (const auto& robot : our_team) {
          if (robot.first == chaser_id_) continue;
          obstacles.add(
              model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
        }
        auto hl = std::make_unique<planner::human_like>();
        hl->set_area(world().field(), field_margin);
        actions.push_back(
            std::make_shared<action::with_planner>(get_ball_, std::move(hl), obstacles));
      }
    }
  }

  // 敵リストの更新
  enemy_list_ = make_enemy_list();

  // followerの更新
  follower_ids_.clear();
  for (const auto& id : ids_) {
    if (!has_chaser_ || !can_chase_ || id != chaser_id_) {
      if (our_team.find(id) != our_team.end()) {
        follower_ids_.push_back(id);
      }
    }
  }

  // マーキングを配置
  update_first_marking();
  update_second_mariking();
  // 余ったロボットを配置
  update_recievers();

  // 1枚目のマーキング
  for (const auto& m : first_marking_) {
    auto hl = std::make_unique<planner::human_like>();
    hl->set_area(world().field(), field_margin);
    planner::obstacle_list obstacles = common_obstacles;
    for (const auto& robot : our_team) {
      if (robot.first == m.first) continue;
      obstacles.add(model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    actions.push_back(std::make_shared<action::with_planner>(first_marking_[m.first],
                                                             std::move(hl), obstacles));
  }
  // 2枚目のマーキング
  for (const auto& m : second_marking_) {
    auto hl = std::make_unique<planner::human_like>();
    hl->set_area(world().field(), field_margin);
    planner::obstacle_list obstacles = common_obstacles;
    for (const auto& robot : our_team) {
      if (robot.first == m.first) continue;
      obstacles.add(model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    actions.push_back(std::make_shared<action::with_planner>(second_marking_[m.first],
                                                             std::move(hl), obstacles));
  }
  // パス受け取り
  for (const auto& r : receive_) {
    auto hl = std::make_unique<planner::human_like>();
    hl->set_area(world().field(), field_margin);
    planner::obstacle_list obstacles = common_obstacles;
    for (const auto& robot : our_team) {
      if (robot.first == r.first) continue;
      obstacles.add(model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    actions.push_back(
        std::make_shared<action::with_planner>(receive_[r.first], std::move(hl), obstacles));
  }
  // 余ったロボット
  for (const auto& m : move_) {
    auto hl = std::make_unique<planner::human_like>();
    hl->set_area(world().field(), field_margin);
    planner::obstacle_list obstacles = common_obstacles;
    for (const auto& robot : our_team) {
      if (robot.first == m.first) continue;
      obstacles.add(model::obstacle::point{util::math::position(robot.second), obs_robot_rad});
    }
    actions.push_back(
        std::make_shared<action::with_planner>(move_[m.first], std::move(hl), obstacles));
  }

  return actions;
}

// chase id_の候補を取得
unsigned int regular::select_chaser() const {
  const auto ball     = world().ball();
  const auto our_team = model::our_robots(world(), team_color());
  std::vector<unsigned int> ids;

  // 見えるロボットだけ選考する
  for (const auto& id : ids_) {
    if (our_team.find(id) != our_team.end()) {
      ids.push_back(id);
    }
  }

  // ボールから最も近いロボットを取得
  auto tmp_id = nearest_id(ids, ball.x(), ball.y());

  if ( // 取得できたか
      tmp_id != ids.end() &&
      // パスの受け取り手ではないか
      receive_.find(*tmp_id) == receive_.end()) {
    return *tmp_id;
  }
  return chaser_id_;
}

// 1枚目のマーキングの再構成
void regular::update_first_marking() {
  first_marking_.clear();

  // no_mark のときはマークしない
  if (mark_option_ == mark_option::no_mark) {
    return;
  }

  const auto those_team = model::enemy_robots(world(), team_color());

  std::unordered_map<unsigned int, unsigned int> new_marked_list;

  // == マーキングを維持させる

  auto old_enemies = enemy_list_;
  while (
      // 味方ロボットが余っているか
      new_marked_list.size() < follower_ids_.size() &&
      //　マークできていない敵がいるか
      !old_enemies.empty()) {
    //　敵のID
    const auto enemy_id = old_enemies.top().id;

    //　敵が一枚目のマーキングにマークされていたか
    auto id_itr = marked_list_.find(enemy_id);
    if (id_itr != marked_list_.end()) {
      // その敵をマークしていたロボットのID
      auto id = id_itr->second;

      // ロボットの手が空いているか
      auto find_itr = std::find(follower_ids_.begin(), follower_ids_.end(), id);
      if (find_itr != follower_ids_.end()) {
        first_marking_[id] = make_action<action::marking>(id);
        first_marking_.at(id)->mark_robot(enemy_id);
        first_marking_.at(id)->set_mode(action::marking::mark_mode::shoot_block);
        first_marking_.at(id)->set_radius(600.0);

        // マーク済みリストに追加
        new_marked_list[enemy_id] = id;

        follower_ids_.erase(find_itr);
      }
    }
    old_enemies.pop();
  }

  // === 新規マーキング

  auto tmp_enemies = enemy_list_;
  while (!follower_ids_.empty() && !tmp_enemies.empty()) {
    // 敵がマーク済みでない時
    if (new_marked_list.find(tmp_enemies.top().id) == new_marked_list.end()) {
      // 対象から最も近いロボットに割り当てる
      const auto tmp_id = nearest_id(follower_ids_, those_team.at(tmp_enemies.top().id).x(),
                                     those_team.at(tmp_enemies.top().id).y());
      if (tmp_id != follower_ids_.end()) {
        first_marking_[*tmp_id] = make_action<action::marking>(*tmp_id);
        first_marking_.at(*tmp_id)->mark_robot(tmp_enemies.top().id);
        first_marking_.at(*tmp_id)->set_mode(action::marking::mark_mode::shoot_block);
        first_marking_.at(*tmp_id)->set_radius(600.0);

        // マーク済みリストに追加
        new_marked_list[tmp_enemies.top().id] = *tmp_id;
        follower_ids_.erase(tmp_id);
      }
    }
    tmp_enemies.pop();
  }

  // リストの更新
  marked_list_ = new_marked_list;
}

// 2枚目のマーキングの再構成
void regular::update_second_mariking() {
  second_marking_.clear();

  // no_mark のときはマークしない
  if (mark_option_ == mark_option::no_mark) {
    return;
  }

  const auto those_team = model::enemy_robots(world(), team_color());
  const auto ball       = world().ball();
  auto tmp_enemies      = enemy_list_;

  // 全ての味方ロボットが割り当てられるか、敵を全てマークするまで続ける
  while (!follower_ids_.empty() && !tmp_enemies.empty()) {
    //自チーム側のゴール->敵ロボの角度
    const double goal_angle =
        std::atan2(0.0 - those_team.at(tmp_enemies.top().id).y(),
                   world().field().x_min() - those_team.at(tmp_enemies.top().id).x());
    //ボール->敵ロボの角度
    const double ball_angle = std::atan2(ball.y() - those_team.at(tmp_enemies.top().id).y(),
                                         ball.x() - those_team.at(tmp_enemies.top().id).x());
    // 一枚目のマーキングと重ならないとき
    if (std::abs(goal_angle - ball_angle) > std::atan2(1.2, 3.0)) {
      const auto tmp_id = nearest_id(follower_ids_, those_team.at(tmp_enemies.top().id).x(),
                                     those_team.at(tmp_enemies.top().id).y());
      if (tmp_id != follower_ids_.end()) {
        // マーキングの設定
        second_marking_[*tmp_id] = make_action<action::marking>(*tmp_id);
        second_marking_.at(*tmp_id)->mark_robot(tmp_enemies.top().id);
        second_marking_.at(*tmp_id)->set_mode(action::marking::mark_mode::kick_block);
        second_marking_.at(*tmp_id)->set_radius(600.0);
        follower_ids_.erase(tmp_id);
      }
    }
    tmp_enemies.pop();
  }
}

//　余ったロボットへの割当て
void regular::update_recievers() {
  move_.clear();
  reserved_points_ = reserved_points_old_;
  std::unordered_map<unsigned int, point> tmp_reserved_points;

  const auto ball       = world().ball();
  const auto our_team   = model::our_robots(world(), team_color());
  const auto those_team = model::enemy_robots(world(), team_color());

  // エリアの先頭
  double area_top;
  // エリア先頭のX座標の下限値
  const double area_top_min = world().field().x_max() * 0.4;

  // receive_の更新用
  std::unordered_map<unsigned int, std::shared_ptr<action::receive>> new_receive;

  // ボールを追いかけるロボットが見えるときはそのロボットを、できないときはボールを判定材料にする
  double top = (has_chaser_ && can_chase_ && our_team.find(chaser_id_) != our_team.end()
                    ? our_team.at(chaser_id_).x()
                    : ball.x());
  if (top <= area_top_min) {
    //　一定以上後ろに下がらないように
    area_top = area_top_min;
  } else if (has_chaser_ && can_chase_) {
    //　chaserがいるときは、パス待ちをするために、少し前へ
    area_top = world().field().x_max() * 0.5;
  } else {
    //　目一杯前にいく
    area_top = world().field().x_max() - world().field().penalty_length() - 300;
  }

  // action生成
  for (const auto& id : follower_ids_) {
    // ロボットがいられる範囲
    area field{area_top, world().field().y_min() + 200.0,
               world().field().x_min() + world().field().penalty_length() + 300,
               world().field().y_max() - 200.0, 0.0};
    // 空いている場所を探す
    auto a = empty_area(field, id, follower_ids_);
    // デフォルトの移動場所は、空いている場所の中心
    regular::point p{(a.x1 + a.x2) / 2, (a.y1 + a.y2) / 2};

    // 最低限ボールから離れる距離
    const double margin = 600.0;
    // ボールから現在の移動点までの距離と角度
    double ball_p       = std::hypot(ball.x() - p.x(), ball.y() - p.y());
    double ball_p_theta = std::atan2(ball.y() - p.y(), ball.x() - p.x());

    if (!has_chaser_ && ball_p < margin) {
      // ボールから離れる
      p.x(p.x() - (margin - ball_p) * std::cos(ball_p_theta));
      p.y(p.y() - (margin - ball_p) * std::sin(ball_p_theta));
    }

    if (our_team.find(id) != our_team.end()) {
      if ( // ボールを持つロボットがいるか
          has_chaser_ && can_chase_ &&
          // パスを受け取れる場所にいるか
          std::hypot(our_team.at(id).x() - target_.x(), our_team.at(id).y() - target_.y()) <
              1500.0 &&
          // ボールを持つロボットが見えるか
          our_team.find(chaser_id_) != our_team.end()) {
        // パスを待機
        auto itr     = receive_.find(id);
        auto receive = itr == receive_.end() ? make_action<action::receive>(id) : itr->second;
        receive->set_passer(chaser_id_);
        new_receive[id] = receive;
      } else {
        // 移動するだけ
        auto move = make_action<action::move>(id);
        move->move_to(p.x(), p.y(), ball_p_theta);
        move_[id] = move;
      }
    }
    // ロボットが移動する先を予約する
    reserved_points_[id]    = p;
    tmp_reserved_points[id] = p;
  }

  receive_ = new_receive;
  // 保持用
  reserved_points_old_ = tmp_reserved_points;
}

// 敵リストの作成
std::priority_queue<regular::id_importance> regular::make_enemy_list() const {
  // 一時的なリスト
  std::priority_queue<regular::id_importance> tmp_enemies;
  // 味方
  const auto our_team = model::our_robots(world(), team_color());
  // 敵
  const auto those_team = model::enemy_robots(world(), team_color());
  // ボール
  const auto ball = world().ball();

  // ペナルティエリアの高さ
  const auto penalty_length = world().field().penalty_length();
  // ペナルティエリアの幅
  const auto penalty_width = world().field().penalty_width();
  // ペナルティエリアとのマージン
  const double margin = 500;
  // 自分側のペナルティエリア
  regular::area my_penalty_area{world().field().x_min(), -penalty_width / 2 - margin,
                                world().field().x_min() + penalty_length + margin,
                                penalty_width / 2 + margin, 0.0};

  for (const auto& that_rob : those_team) {
    // 敵ロボットのID
    const auto id = that_rob.first;
    // 敵ロボットの位置
    const regular::point that_p{that_rob.second.x(), that_rob.second.y()};
    // ロボットのx速度
    const double robot_vx = that_rob.second.vx();
    // ゴールとの距離
    const double gall_dist = std::hypot(that_p.x() - world().field().x_min(), that_p.y());
    // ボールとの距離
    const double ball_dist = std::hypot(that_p.x() - ball.x(), that_p.y() - ball.y());
    // ゴールからの角度
    const double gall_angle = std::atan2(that_p.y(), that_p.x() - world().field().x_min());
    // ゴールからみたボールの角度
    const double ball_gall_angle = std::atan2(ball.y(), ball.x() - world().field().x_min());

    // ボールを持っているか
    bool is_kicker = ball_dist < 1000;

    // chaserにマークを委託
    if (has_chaser_ && can_chase_ && is_kicker) continue;

    // 対象となるエリアの中か
    bool is_marking_area =
        // 敵側に寄りすぎていないか
        that_p.x() < world().field().x_max() * 0.65 &&
        // ペナルティエリア付近にいないか
        !regular::in_area(that_p.x(), that_p.y(), my_penalty_area) &&
        // ディフェンスの近くにいないか
        !(std::abs(ball_gall_angle - gall_angle) < atan2(1.2, 4.0) && gall_dist < 4000);

    // マーキングエリア内 && chaserとかぶらない時
    if (is_marking_area) {
      // 敵のマーク重要度
      double score = 0.0;
      // 距離の正規化用に使う
      const double gall_dist_max = std::hypot(world().field().x_max() - world().field().x_min(),
                                              world().field().y_max());

      // ゴールに近い or ゴールに対して正面なほど優先度高め
      score += std::cos(gall_angle * 0.3) * (gall_dist_max - gall_dist) / gall_dist_max;

      // ある一定以上のスピードで移動している時
      if (std::abs(robot_vx) > 1100.0) {
        // こっち側に向かってくる程優先度高め
        // 爆走してきた時に優先度TOPになるように
        // -robot_vx/<爆走した時の速度>*(<重要度を覆したい分のx幅>/gall_dist_max）
        score -= robot_vx / 5000 * 0.8 * (world().field().x_max() - world().field().x_min()) /
                 gall_dist_max;
      }

      tmp_enemies.push({id, score});
    }
  }

  // マーキングのロボットが密集しないように、密集の元になる敵はリストから除外
  std::vector<unsigned int> added_list;
  std::priority_queue<regular::id_importance> enemies;

  pop_each(tmp_enemies, [&those_team, &enemies, &added_list, this](const auto& item) {
    // 敵ロボのID
    const auto id = item.id;
    // 敵ロボの位置
    const regular::point that_p{those_team.at(id).x(), those_team.at(id).y()};

    if (added_list.empty()) {
      // 初めて選ばれた要素のとき
      enemies.push(item);
      added_list.push_back(id);
    } else {
      double target_theta = std::atan2(that_p.y(), that_p.x() - world().field().x_min());

      // 最も角度差が小さいID
      auto next_id = std::min_element(
          added_list.begin(), added_list.end(),
          [&target_theta, &those_team, this](unsigned int a, unsigned int b) {
            const double theta_a = std::atan2(those_team.at(a).y(),
                                              those_team.at(a).x() - world().field().x_min());
            const double theta_b = std::atan2(those_team.at(b).y(),
                                              those_team.at(b).x() - world().field().x_min());
            return std::abs(theta_a - target_theta) < std::abs(theta_b - target_theta);
          });

      if (next_id != added_list.end()) {
        const regular::point next_p{those_team.at(*next_id).x(), those_team.at(*next_id).y()};
        // 最も角度が近いロボットとの角度差
        const auto smallest_theta =
            std::abs(std::atan2(that_p.y(), that_p.x() - world().field().x_min()) -
                     std::atan2(next_p.y(), next_p.x() - world().field().x_min()));

        if (smallest_theta > std::atan2(1.0, 2.8)) {
          // ある程度の角度差が保たれている時
          enemies.push(item);
          added_list.push_back(id);
        }
      }
    }
  });
  return enemies;
}

// パス経路を生成してポイントを返す
regular::point regular::select_target() const {
  // ゴール幅
  const auto goal_width = world().field().goal_width();
  // 敵ゴールの左端
  const regular::point goal_left{world().field().x_max(), goal_width / 2 - 150};
  // 敵ゴールの右端
  const regular::point goal_right{world().field().x_max(), -goal_width / 2 + 150};
  // 敵ゴールの中心
  const regular::point goal_center{world().field().x_max(), 0.0};
  // 味方
  const auto our_team = model::our_robots(world(), team_color());
  // 敵
  const auto those_team = model::enemy_robots(world(), team_color());
  // ボール
  const auto ball = world().ball();

  // 敵(ディフェンス、キーパー以外)の位置
  std::vector<regular::point> enemy_points;
  // 敵ディフェンスの位置
  std::vector<regular::point> defense_points;
  // シュート先候補の位置
  std::vector<regular::point> shoot_points;

  // 敵の位置 & 敵ディフェンス の追加
  {
    // ペナルティエリアの幅
    const auto penalty_width = world().field().penalty_width();
    // ペナルティエリアの高さ
    const auto penalty_length = world().field().penalty_length();
    // マージン
    const double margin = 400;
    // ディフェンスがいるエリア
    const regular::area d_area{world().field().x_max(), -penalty_width / 2 - margin,
                               world().field().x_max() - penalty_length - margin,
                               penalty_width / 2 + margin, 0.0};

    for (const auto& that_rob : those_team) {
      regular::point p{that_rob.second.x(), that_rob.second.y()};

      if (regular::in_area(p.x(), p.y(), d_area)) {
        defense_points.push_back(p);
      } else {
        enemy_points.push_back(p);
      }
    }
  }

  //  == シュート先の候補を追加

  // ゴール中心は問答無用で候補に
  shoot_points.push_back(goal_center);

  // ゴールからの距離
  double ball_goal = std::hypot(goal_center.x() - ball.x(), goal_center.y() - ball.y());
  // ゴールに対する角度
  double ball_goal_theta = std::atan2(ball.y(), goal_center.x() - ball.x());
  // シュートが可能か
  bool can_shoot = std::abs(ball_goal_theta) < std::atan2(1.7, 1);

  // ゴールの中心以外にもシュートできるとき
  if (can_shoot && ball_goal < 4300) {
    shoot_points.push_back(goal_left);
    shoot_points.push_back(goal_right);
  }

  // == 最適なシュートコースを求める

  // デフォルトはゴール中心にする
  point shoot_p = goal_center;

  // 敵ディフェンスがいる時
  if (!defense_points.empty()) {
    double max_theta = 0.0;

    for (const auto& p : shoot_points) {
      // シュート先-＞ボールの角度
      double p_ball_theta = std::atan2(p.y() - ball.y(), p.x() - ball.x());

      // 最も邪魔な敵の位置
      const auto defense_p = std::min_element(
          defense_points.begin(), defense_points.end(),
          [&ball, &p_ball_theta](regular::point a, regular::point b) {
            double theta_a = std::atan2(a.y() - ball.y(), a.x() - ball.x());
            double theta_b = std::atan2(b.y() - ball.y(), b.x() - ball.x());
            return std::abs(theta_a - p_ball_theta) < std::abs(theta_b - p_ball_theta);
          });

      if (defense_p != defense_points.end()) {
        // 敵-＞ボールの角度
        double defense_ball_theta =
            std::atan2(defense_p->y() - ball.y(), defense_p->x() - ball.x());

        // より角度差が大きい位置を選ぶ
        if (std::abs(p_ball_theta - defense_ball_theta) > max_theta) {
          shoot_p   = p;
          max_theta = std::abs(p_ball_theta - defense_ball_theta);
        }
      }
    }
  }

  // パス先候補の位置
  std::vector<regular::point> pass_points;
  // パス先候補の追加
  for (const auto& p : reserved_points_) {
    if (( // ボールより前にいるか
            p.second.x() > ball.x() ||
            // フィールドの敵側にある程度寄っているか
            p.second.x() > 0.0) &&
        // 打つ場所から離れているか
        std::hypot(p.second.x() - ball.x(), p.second.y() - ball.y()) > 3000) {
      pass_points.push_back(p.second);
    }
  }

  // == 以下の条件を満たしたら必ずシュート
  if ( // パスの相手がいない
      pass_points.empty() ||
      // パスしないほうが有利な場所
      ball.x() < world().field().x_max() * 0.5 ||
      // ゴールからほぼ真正面
      std::abs(std::atan2(ball.y(), goal_center.x() - ball.x())) < std::atan2(1, 2) ||
      // シュートしやすい位置
      (can_shoot &&
       std::hypot(goal_center.x() - ball.x(), goal_center.y() - ball.y()) < 3000) ||
      // 敵がいなくて角度も問題ない
      (enemy_points.empty() && can_shoot)) {
    return shoot_p;
  }

  // == パスも含めた最適コースを求める

  // 候補
  auto kick_points = pass_points;

  // シュート可能なら追加
  if (can_shoot) {
    kick_points.insert(kick_points.end(), shoot_points.begin(), shoot_points.end());
  }

  // 蹴る先の保持
  regular::point kick_p = can_shoot ? shoot_p : pass_points.at(0);

  // 敵がいない時
  if (enemy_points.empty()) {
    // 一番通りやすいところにキック
    auto p = std::min_element(
        kick_points.begin(), kick_points.end(), [this](regular::point a, regular::point b) {
          double theta_a = std::atan2(a.y(), world().field().x_max() - a.x());
          double theta_b = std::atan2(b.y(), world().field().x_max() - b.x());
          return std::abs(theta_a) < std::abs(theta_b);
        });
    if (p != enemy_points.end()) {
      return *p;
    }
  }

  // 蹴る先と最も邪魔な敵との角度差の最大値
  double max_theta = 0.0;

  // すべてのコースから最適解を求める
  for (const auto& kp : kick_points) {
    // キック先-＞ボールの距離
    double kp_ball = std::hypot(ball.x() - kp.x(), ball.y() - kp.y());
    // キック先-＞ボールの角度
    double kp_angle = std::atan2(kp.y() - ball.y(), kp.x() - ball.x());

    // 邪魔な敵の候補
    std::vector<regular::point> tmp_ep;
    for (const auto& ep : enemy_points) {
      if (std::hypot(ep.x() - ball.x(), ep.y() - ball.y()) <= kp_ball) {
        tmp_ep.push_back(ep);
      }
    }

    // 邪魔な敵がいない時
    if (tmp_ep.empty()) {
      return kp;
    }

    // 最も邪魔な敵の位置を探す
    const auto ep = std::min_element(
        tmp_ep.begin(), tmp_ep.end(), [&ball, &kp_angle](regular::point a, regular::point b) {
          double theta_a = std::atan2(a.y() - ball.y(), a.x() - ball.x());
          double theta_b = std::atan2(b.y() - ball.y(), b.x() - ball.x());
          return std::abs(theta_a - kp_angle) < std::abs(theta_b - kp_angle);
        });

    if (ep != tmp_ep.end()) {
      // 敵-＞ボールの角度
      double enemy_ball_theta = std::atan2(ep->y() - ball.y(), ep->x() - ball.x());
      // より角度差が大きい位置を選ぶ
      if (std::abs(kp_angle - enemy_ball_theta) > max_theta) {
        kick_p    = kp;
        max_theta = std::abs(kp_angle - enemy_ball_theta);
      }
    }
  }
  return kick_p;
}

// ターゲットに最も近いロボットID
std::vector<unsigned int>::const_iterator regular::nearest_id(
    const std::vector<unsigned int>& can_ids, double target_x, double target_y) const {
  const auto our_team = model::our_robots(world(), team_color());
  return std::min_element(
      can_ids.begin(), can_ids.end(),
      [&target_x, &target_y, our_team](unsigned int a, unsigned int b) {
        return std::hypot(our_team.at(a).x() - target_x, our_team.at(a).y() - target_y) <
               std::hypot(our_team.at(b).x() - target_x, our_team.at(b).y() - target_y);
      });
}

// 空いているエリアを返す
regular::area regular::empty_area(const regular::area& area, unsigned int my_id,
                                  const std::vector<unsigned int>& our_ids) const {
  // X方向の分割数
  const int split_x = 3;
  // Y方向の分割数
  const int split_y = split_x;
  // 各エリアの幅
  const double width = (area.x2 - area.x1) / split_x;
  // 各エリアの高さ
  const double height = (area.y2 - area.y1) / split_y;

  // エリア生成
  std::vector<std::vector<regular::area>> areas(split_y, std::vector<regular::area>(split_x));
  for (int i = 0; i < split_y; i++) {
    for (int j = 0; j < split_x; j++) {
      regular::area t;
      t.x1              = area.x1 + j * width;
      t.y1              = area.y1 + i * height;
      t.x2              = t.x1 + width;
      t.y2              = t.y1 + height;
      t.score           = 0.0;
      areas.at(i).at(j) = t;
    }
  }

  // 味方
  const auto our_team = model::our_robots(world(), team_color());
  // 敵
  const auto those_team = model::enemy_robots(world(), team_color());
  // 自分以外のロボットの位置
  std::vector<regular::point> points;

  // 予約済みの位置を追加
  for (const auto& p : reserved_points_) {
    if (p.first != my_id) {
      points.push_back(p.second);
    }
  }

  // == our_idsに含まれていないロボットの現在位置を追加
  // 味方
  for (const auto& robot : our_team) {
    if (std::find(our_ids.begin(), our_ids.end(), robot.first) == our_ids.end()) {
      regular::point p{robot.second.x(), robot.second.y()};
      points.push_back(p);
    }
  }
  //敵
  for (const auto& robot : those_team) {
    regular::point p{robot.second.x(), robot.second.y()};
    points.push_back(p);
  }

  // 得点
  const double robot_score  = split_x * split_y;
  const double option_score = 1.0;

  // ロボットのいる位置に得点を加算
  for (int i = 0; i < split_y; i++) {
    if (points.empty()) break;

    for (int j = 0; j < split_x; j++) {
      if (points.empty()) break;

      for (auto&& p = points.end() - 1; p >= points.begin(); p--) {
        // ロボットがいたとき
        if (regular::in_area(p->x(), p->y(), areas.at(i).at(j))) {
          // ロボットがいた場所に得点
          areas.at(i).at(j).score += robot_score;
          points.erase(p);

          // == ロボットの周りの場所に得点

          // 後ろ側
          if (j > 0) {
            // 真後ろ
            areas.at(i).at(j - 1).score += option_score;
            // 右後方
            if (i > 0) areas.at(i - 1).at(j - 1).score += option_score;
            // 左後方
            if (i + 1 < split_y) areas.at(i + 1).at(j - 1).score += option_score;
          }

          // 前側
          if (j + 1 < split_x) {
            // 真正面
            areas.at(i).at(j + 1).score += option_score;
            // 右前方
            if (i > 0) areas.at(i - 1).at(j + 1).score += option_score;
            // 左前方
            if (i + 1 < split_y) areas.at(i + 1).at(j + 1).score += option_score;
          }

          // 右
          if (i > 0) areas.at(i - 1).at(j).score += option_score;
          // 左
          if (i + 1 < split_y) areas.at(i + 1).at(j).score += option_score;
        }
      }
    }
  }

  // スコアが最も低い場所を取り出す
  bool is_first = true;
  regular::area tmp_area;
  for (const auto& i : areas) {
    for (const auto& j : i) {
      if (is_first || j.score < tmp_area.score) tmp_area = j;
      is_first = false;
    }
  }

  // 最小のスコアが0より大きいなら再帰
  return tmp_area.score > 0 ? empty_area(tmp_area, my_id, our_ids) : tmp_area;
}

// 指定位置がエリア上にあるかどうか
bool regular::in_area(double x, double y, const regular::area& area) const {
  // 左上の頂点
  regular::point top_left;
  //右下の頂点
  regular::point bottom_right;

  bool is_right = area.x1 <= area.x2;
  bool is_down  = area.y1 <= area.y2;

  // 向きを合わせる
  top_left.x((is_right ? area.x1 : area.x2));
  top_left.y((is_down ? area.y1 : area.y2));
  bottom_right.x((is_right ? area.x2 : area.x1));
  bottom_right.y((is_down ? area.y2 : area.y1));

  // 指定点
  regular::point p{x, y};
  // エリア
  boost::geometry::model::box<regular::point> box{top_left, bottom_right};
  return boost::geometry::within(p, box);
}

bool regular::id_importance::operator<(const id_importance& next) const {
  // 重要度で比較
  return importance < next.importance;
}

} // namespace ai_server::game::agent
