#include "regular.h"
#include "ai_server/model/robot.h"

namespace ai_server {
namespace game {
namespace agent {

regular::regular(const model::world& world, bool is_yellow,
                 const std::vector<unsigned int>& ids)
    : base(world, is_yellow), ids_(ids) {
  ball_chase_ = true; //デフォルトではtrue

  chase_ball_finished_  = false;
  kick_action_finished_ = false;


  //  regular::set_robot_ids_zone_ids();
  //  regular::set_robot_zone_ids_();
  unsigned int chase_ball_robot_id = ids.at(0);

  chase_ball_  = std::make_shared<action::chase_ball>(world_, is_yellow_, chase_ball_robot_id);
  kick_action_ = std::make_shared<action::kick_action>(world_, is_yellow_, chase_ball_robot_id);
}

void regular::set_ball_chase(bool ball_chase) {
  if (ball_chase_ != ball_chase) {
    regular::set_zones_();
  }
  ball_chase_ = ball_chase;
}

std::vector<std::shared_ptr<action::base>> regular::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
  regular::set_zones_();
  const auto ball         = world_.ball();
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();

  std::vector<unsigned int> those_ids; //敵ロボットのID
  int chase_ball_robot_id = -1; // ball_chase_==trueなら担当ロボットのゾーンID、falseなら-1

  //敵ロボットのIDを登録
  for (auto that_rob : those_robots) {
    those_ids.push_back(that_rob.first);
  }

  //ボールを追いかけるロボット
  if (ball_chase_) {
    chase_ball_robot_id = ids_.at(0);
    //    regular::robot_id_to_zone_id_(regular::point_to_zone_id_(ball.x(), ball.y()));

    std::printf("%s   %s\n\n",
                chase_ball_finished_ ? "\x1b[45m*ChaseBall*\x1b[49m" : "ChaseBall",
                kick_action_finished_ ? "\x1b[46m*Kick*\x1b[49m" : "Kick");

    if (chase_ball_finished_ && kick_action_finished_) {
      chase_ball_finished_  = false;
      kick_action_finished_ = false;
      std::printf(">>>>>>>>>>> ReTry <<<<<<<<<<\n");
    } else if (chase_ball_finished_) {
      kick_action_->kick_to(world_.field().x_max(), 0.0);
      kick_action_->set_kick_type(std::tuple<model::command::kick_type_t, double>(model::command::kick_type_t::line, 50.0));
      kick_action_->set_mode(action::kick_action::mode::goal);
      kick_action_finished_ = kick_action_->finished();
      actions.push_back(kick_action_);
    } else {
      chase_ball_->set_target(world_.field().x_max(), 0.0);
      chase_ball_finished_ = chase_ball_->finished();
      actions.push_back(chase_ball_);
    }
    actions.push_back(chase_ball_);
  } else {
    chase_ball_finished_  = false;
    kick_action_finished_ = false;
  }

  //  --- マーキングを担当するロボットを割り当て ---

  //ゾーン優先度を決める
  std::vector<id_importance_> zone_importance;
  for (unsigned int i = 0; i < zones_.size(); i++) {
    zone_importance.push_back({i, 0});
  }

  //敵の集まり具合 --> ゾーンのマーク重要度
  //ボールのある所は重要度100足す
  for (auto id : those_ids) {
    int those_zone_id =
        regular::point_to_zone_id_(those_robots.at(id).x(), those_robots.at(id).y());
    zone_importance[those_zone_id].importance++;

    if (ball_chase_) {
      /* if (those_zone_id == robot_id_zone_id_.at(chase_ball_id_)) {
         zone_importance[those_zone_id].importance += 100;
       }*/
    }
  }

  id_id_ those_robot_zone_ids;
  std::vector<id_importance_> those_id_importance;

  // -- 敵の優先度を決める --
  for (unsigned int that_id : those_ids) {
    those_id_importance.push_back({that_id, -1.0 * those_robots.at(that_id).x()});
    those_robot_zone_ids[that_id] =
        point_to_zone_id_(those_robots.at(that_id).x(), those_robots.at(that_id).y());
  }
  std::sort(those_id_importance.begin(), those_id_importance.end()); //ソート

  /*
      for (int i = 0; i < zones_.size(); i++) {

                  marking_ = std::make_shared<action::marking>(world_, zone_id_to_robot_id_(i));

        if (zone_importance.at(i).importance == 0.0) {

          //敵がいない時は、ゾーンの両側を見て、敵が多い方につく
          //(敵コートならパスカット、味方コートならシュートカット)
          if (i != 0 && i + 1 < zones_.size()) {

            if (zone_importance.at(i - 1).importance > zone_importance.at(i - 1).importance) {

              marking_->mark_robot(zone_id_to_robot_id_x_(i-1,those_robot_zone_ids));
              marking_->set_mode(action::marking::mark_mode::kick_block);
            } else {
               marking_->mark_robot(zone_id_to_robot_id_x_(i+1,those_robot_zone_ids));
              marking_->set_mode(action::marking::mark_mode::kick_block);
            }
          } else if (i > 0) {
            marking_->mark_robot(zone_id_to_robot_id_x_(i+1,those_robot_zone_ids));
            marking_->set_mode(action::marking::mark_mode::kick_block);


          } else {
            marking_->mark_robot(zone_id_to_robot_id_x_(i-1,those_robot_zone_ids));
          }
                  marking_->set_mode(action::marking::mark_mode::kick_block);
                  marking_->set_radius(1000.0);

        } else if (zone_importance.at(i).importance > 1.0) {
          //敵が一人以上の時
          //(敵コートならパスカット、味方コートならシュートカット)
          marking_->mark_robot(i);
          marking_->set_mode(action::marking::mark_mode::kick_block);
          marking_->set_radius(300.0);

        } else {
          //敵が一人の時
          //シュートカット
          marking_->mark_robot(i);
          marking_->set_mode(action::marking::mark_mode::kick_block);
          marking_->set_radius(300.0);
        }
              actions.push_back(marking_);
        // std::printf("\n\x1b[32m***** ***** One Loop ***** *****\x1b[39m\n\n"); // debug
      }
    */

  return actions;
}

//ボールを追いかけるロボットを設定
void regular::set_chase_ball_id_() {
  const auto ball   = world_.ball();
  const auto robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
}

///ゾーンの設定
void regular::set_zones_() {
  zones_.erase(zones_.begin(), zones_.end()); // 再設定
  regular::zone_ zone;
  double zone_height =
      std::abs(world_.field().y_max() - world_.field().y_min()) / ids_.size();
  //ロボット台数に応じたゾーンを設定
  for (unsigned int i = 0; i < ids_.size(); i++) {
    zone = {i, world_.field().x_min() , world_.field().y_max()- i * zone_height,
            world_.field().x_max() , world_.field().y_max()- (i + 1) * zone_height};
    zones_.push_back(zone);
    std::printf("Zone ID:%d - x:%.0f y:%.0f x-max:%.0f y-max:%.0f\n", zone.id, zone.x_min, zone.y_min,
                zone.x_max, zone.y_max);
  }
}

///ロボットのIDと守るべきゾーンIDの関連付け
void regular::set_robot_ids_zone_ids_() {
  id_id_ robot_place;
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();

  std::vector<std::vector<unsigned int>> zone_order = {
      ids_.size(), std::vector<unsigned int>(ids_.size(), 0)};
  std::vector<std::vector<unsigned int>> robot_order;

  robot_id_zone_id_.erase(robot_id_zone_id_.begin(), robot_id_zone_id_.end()); // 再設定用

  //ロボットの位置を取得
  for (auto robot_id : ids_) {
    for (auto zone : zones_) {
      if (regular::on_area_(ids_robots.at(robot_id).x(), ids_robots.at(robot_id).y(), zone)) {
        robot_place[robot_id] = zone.id;
        break;
      }
    }
  }

  //ロボットの希望リストを生成
  for (auto&& robot_id : ids_) {
    
    unsigned int zone_id = robot_place.at(robot_id);
    std::vector<unsigned int> tmp_order;
    
    for (unsigned int d = 0; tmp_order.size() < zones_.size(); d++) {
      
      if (zone_id + d < zones_.size()) {
        tmp_order.push_back(zone_id+d);
      }
      
      if (tmp_order.size() < zones_.size() && zone_id - d >= 0 && d > 0) {
                tmp_order.push_back(zone_id-d);
      }
      
    }
    
    robot_order.push_back(tmp_order);
  }

  //ゾーンの希望準リストを生成
  for (unsigned int i=0;i<zones_.size();i++) {
    
    break;
  }

  //マッチング結果を取得
  std::vector<unsigned int> match_zone_ids = regular::stable_matching_(robot_order, zone_order);
  for (int i = 0; i < ids_.size(); i++) {
    robot_id_zone_id_[ids_.at(i)] = match_zone_ids.at(i);
    std::printf("robot \x1b[44m'%d'\x1b[49m go to zone \x1b[44m'%d'\x1b[49m\n", ids_.at(i),
                match_zone_ids.at(i)); /// debug
  }
}

///安定マッチング
/// arguments : 男性側の希望順リスト,女性の希望順リスト
/// return    : 配列要素が女性、添字(index)は男性   --> 男性の指定順は保持される
std::vector<unsigned int> regular::stable_matching_(
    const std::vector<std::vector<unsigned int>>& order_man,
    const std::vector<std::vector<unsigned int>>& order_woman) {
  const unsigned int n = order_man.size();
  std::vector<std::vector<unsigned int>> prefer_woman(n, std::vector<unsigned int>(n + 1, n));
  std::vector<unsigned int> match_woman(n, n);
  std::vector<unsigned int> proposed_man(n);

  for (unsigned int w = 0; w < n; w++) {
    for (unsigned int i = 0; i < n; i++) {
      prefer_woman[w][order_woman[w][i]] = i;
    }
  }

  for (unsigned int m_ = 0; m_ < n; m_++) {
    for (unsigned int m = m_; m < n;) {
      unsigned int w = order_man[m][proposed_man[m]++];
      if (prefer_woman[w][m] < prefer_woman[w][match_woman[w]]) {
        std::swap(m, match_woman[w]);
      }
    }
  }
  return match_woman;
}

///ある点が指定されたエリア上にあるか(第2引数のzone_area_構造体のIDは判定に使われない)
bool regular::on_area_(double x, double y, regular::zone_ zone) {
  return x >= zone.x_min && y >= zone.y_min && x <= zone.x_max && y <= zone.y_max;
}

///ある地点に対応するゾーンIDを返す
unsigned int regular::point_to_zone_id_(double x, double y) {
  unsigned int i = 0;
  for (i = 0; i < zones_.size(); i++) {
    if (regular::on_area_(x, y, zones_.at(i))) {
      break;
    }
  }
  return i;
}

} // agent
} // game
} // ai_server
