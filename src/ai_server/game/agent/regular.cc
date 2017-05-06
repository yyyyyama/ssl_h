#include "regular.h"
#include "ai_server/model/robot.h"

namespace ai_server
{
namespace game
{
namespace agent
{

regular::regular(const model::world &world, bool is_yellow,
                 const std::vector<unsigned int> &ids)
    : base(world, is_yellow), ids_(ids)
{
  ball_chase_ = true;

  chase_ball_finished_ = false;
  kick_action_finished_ = false;

  //  regular::set_zones_();
  //  regular::set_robot_ids_zone_ids();
  //  regular::set_robot_zone_ids_();
  unsigned int chase_ball_robot_id = ids.at(0);

  chase_ball_ = std::make_shared<action::chase_ball>(world_, is_yellow_, chase_ball_robot_id);
  kick_action_ = std::make_shared<action::kick_action>(world_, is_yellow_, chase_ball_robot_id);
}

void regular::set_ball_chase(bool ball_chase)
{
  ball_chase_ = ball_chase;
  // regular::set_zones_();
}

std::vector<std::shared_ptr<action::base>> regular::execute()
{
  std::vector<std::shared_ptr<action::base>> actions;

  const auto ball = world_.ball();
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();

  std::vector<unsigned int> those_ids; //敵ロボットのID
  int chase_ball_robot_id = -1;        // ball_chase_==trueなら担当ロボットのゾーンID、falseなら-1

  //敵ロボットのIDを登録
  for (auto that_rob : those_robots)
  {
    those_ids.push_back(that_rob.first);
  }

  //ボールを追いかけるロボット
  if (ball_chase_) {
    chase_ball_robot_id = ids_.at(0);
    //    regular::robot_id_to_zone_id_(regular::point_to_zone_id_(ball.x(), ball.y()));

    std::printf("%s   %s\n\n", chase_ball_finished_ ? "\x1b[45mChaseBall\x1b[49m" : "ChaseBall",
                kick_action_finished_ ? "\x1b[46mKick\x1b[49m" : "Kick");

      if (chase_ball_finished_ && kick_action_finished_)
      {

        chase_ball_finished_ = false;
        kick_action_finished_ = false;
        std::printf("\x1b[44m                                                       \n                         Retry                         \n                                                       \x1b[49m\n\n");
      }
      else if (chase_ball_finished_)
      {
        kick_action_->kick_to(world_.field().x_max(), 0.0);
        kick_action_->set_kick_type(std::tuple<model::command::kick_type_t, double>(
            model::command::kick_type_t::line, 50.0));
        kick_action_->set_mode(action::kick_action::mode::goal);
        kick_action_finished_ = kick_action_->finished();
        actions.push_back(kick_action_);
      }
      else
      {
        chase_ball_->set_target(world_.field().x_max(), 0.0);
        chase_ball_finished_ = chase_ball_->finished();
        actions.push_back(chase_ball_);
      }
      actions.push_back(chase_ball_);
  }
  else
  {
    chase_ball_finished_ = false;
    kick_action_finished_ = false;
  }

  /*
    //--- マーキングを担当するロボットを割り当て ---

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
      zone_importance.at(those_zone_id).importance++;

      if (chase_ball_robot_id > 0) {
        if (those_zone_id == robot_id_to_zone_id_(chase_ball_robot_id)) {
          zone_importance.at(those_zone_id).importance += 100;
        }
      }
    }

    //
    std::vector<robot_zone_id_> those_robot_zone_ids;

    //敵の優先度を決める
    std::vector<id_importance_> those_id_importance;
    for (unsigned int id : those_ids) {
      those_id_importance.push_back({id, -1.0 * those_robots.at(id).x()});
      //
      those_robot_zone_ids.push_back(
          {id, point_to_zone_id_(those_robots.at(id).x(), those_robots.at(id).y())});
    }
    std::sort(those_id_importance.begin(), those_id_importance.end()); //ソート

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
void regular::set_chase_ball_id_()
{
  const auto ball = world_.ball();
  const auto robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  for (auto &&r_id : ids_)
  {
  }
}

//ゾーンの設定
void regular::set_zones_()
{
  zones_.erase(zones_.begin(), zones_.end()); //再設定ができるようにしておく
  regular::zone_ zone;
  //ロボット台数に応じたゾーンを設定
  for (unsigned int i = 0; i < ids_.size(); i++)
  {
    zone = {i, world_.field().x_min(), world_.field().y_max(),
            world_.field().x_max() * (double)i / (double)ids_.size(),
            1.0 / world_.field().y_max() / (double)ids_.size()};
    zones_.push_back(zone);
  }
}

void regular::set_robot_ids_zone_ids()
{
  robot_id_zone_id robot_place;
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();

  std::vector<std::vector<unsigned int>> zone_order = {
      ids_.size(), std::vector<unsigned int>(ids_.size(), 0)};
  std::vector<std::vector<unsigned int>> robot_order = {
      ids_.size(), std::vector<unsigned int>(ids_.size(), 0)};

  robot_zone_ids_.erase(robot_zone_ids_.begin(),
                        robot_zone_ids_.end()); //再設定ができるようにしておく

  //ロボットの位置を取得
  for (auto robot_id : ids_)
  {
    for (auto zone : zones_)
    {
      if (regular::on_area_(ids_robots.at(robot_id).x(), ids_robots.at(robot_id).y(), zone))
      {
        robot_place[robot_id] = zone.id;
        break;
      }
    }
  }

  //マッチングの準備
  unsigned int robot_count = 0;
  for (auto &&robot_id : ids_)
  {
    unsigned int zone_id = robot_place.at(robot_id);
    for (unsigned int a = 0, d = 0; a < zones_.size(); d++)
    {
      if (zone_id + d < zones_.size())
      {
        robot_order[robot_count][a] = zone_id + d;
        a++;
      }
      if (zone_id - d >= 0)
      {
        robot_order[robot_count][a] = zone_id - d;
        a++;
      }
    }
    robot_count++;
  }

  /*

  //マッチング
  std::vector<unsigned int> match_robot_ids = regular::stable_matching(zone_order, robot_order);
  for (unsigned int i = 0; i < match_robot_ids.size(); i++) {
    robot_zone_ids_.push_back({match_robot_ids.at(i), i});
    //    std::printf("robot \x1b[44m%d\x1b[49m go to zone \x1b[44m%d\x1b[49m\n",
    //    match_robot_ids.at(i), i); /// debug
  }*/
}

//ロボットIDとゾーンIDの関連付け
void regular::set_robot_zone_ids_()
{
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  // robot_ids_zone_ids robot_place;
  /*
  std::vector<std::vector<unsigned int>> zone_order = {
      ids_.size(), std::vector<unsigned int>(ids_.size(), 0)};
  std::vector<std::vector<unsigned int>> robot_order = {
      ids_.size(), std::vector<unsigned int>(ids_.size(), 0)};

  robot_zone_ids_.erase(robot_zone_ids_.begin(),
                        robot_zone_ids_.end()); //再設定ができるようにしておく

  //ロボットの位置を取得
  for (auto robot_id : ids_) {
    for (auto zone : zones_) {
      if (regular::on_area_(ids_robots.at(robot_id).x(), ids_robots.at(robot_id).y(), zone)) {
        robot_place.push_back({robot_id, zone.id});
        break;
      }
    }
  }

  //マッチングの準備
  std::sort(robot_place.begin(), robot_place.end()); //ソート --> i==at(i).zone_id

  for (unsigned int i = 0; i < ids_.size(); i++) {
    unsigned int j = 0;
    while (j < ids_.size()) {
      zone_order[i][j] = robot_place.at(j).robot_id;  //ゾーンに近い順(i+0)
      robot_order[robot_place.at(j).robot_id][j] = j; //ロボットに近い順（i+0）
      if (i != j && i - j >= 0) {
        j++;
        zone_order[i][j] = robot_place.at(i - j).robot_id;  //ゾーンに近い順(i-0)
        robot_order[robot_place.at(j).robot_id][j] = i - j; //ロボットに近い順（i-0）
      }
      j++;
    }
  }

  //マッチング
  std::vector<unsigned int> match_robot_ids = regular::stable_matching(zone_order, robot_order);
  for (unsigned int i = 0; i < match_robot_ids.size(); i++) {
    robot_zone_ids_.push_back({match_robot_ids.at(i), i});
    //    std::printf("robot \x1b[44m%d\x1b[49m go to zone \x1b[44m%d\x1b[49m\n",
    //    match_robot_ids.at(i), i); /// debug
  }*/
}

//安定マッチング
std::vector<unsigned int> regular::stable_matching(
    const std::vector<std::vector<unsigned int>> &order_man,
    const std::vector<std::vector<unsigned int>> &order_woman)
{
  const unsigned int n = order_man.size();
  std::vector<std::vector<unsigned int>> prefer_woman(n, std::vector<unsigned int>(n + 1, n));
  std::vector<unsigned int> match_woman(n, n);
  std::vector<unsigned int> proposed_man(n);

  for (unsigned int w = 0; w < n; w++)
  {
    for (unsigned int i = 0; i < n; i++)
    {
      prefer_woman[w][order_woman[w][i]] = i;
    }
  }

  for (unsigned int m_ = 0; m_ < n; m_++)
  {
    for (unsigned int m = m_; m < n;)
    {
      unsigned int w = order_man[m][proposed_man[m]++];
      if (prefer_woman[w][m] < prefer_woman[w][match_woman[w]])
      {
        std::swap(m, match_woman[w]);
      }
    }
  }
  return match_woman;
}

//ある点が指定されたエリア上にあるか(第2引数のzone_area_構造体のIDは関係無い)
bool regular::on_area_(double x, double y, regular::zone_ zone)
{
  return x >= zone.x && y >= zone.y && x <= zone.x + zone.width && y <= zone.y + zone.height;
}

//ゾーンIDに対応したロボットIDを返す
unsigned int regular::zone_id_to_robot_id_(unsigned int id)
{
  unsigned int ret = 0;
  for (unsigned int i = 0; i < zones_.size(); i++)
  {
    if (robot_zone_ids_.at(i).zone_id == id)
    {
      ret = robot_zone_ids_.at(i).robot_id;
      break;
    }
  }
  return ret;
}

//ロボットIDに対応したゾーンIDを返す
unsigned int regular::robot_id_to_zone_id_(unsigned int id)
{
  unsigned int ret = 0;
  for (unsigned int i = 0; i < zones_.size(); i++)
  {
    if (robot_zone_ids_.at(i).robot_id == id)
    {
      ret = robot_zone_ids_.at(i).zone_id;
      break;
    }
  }
  return ret;
}

//ゾーンIDに対応したロボットIDを返す(敵も対応)
unsigned int regular::zone_id_to_robot_id_x_(unsigned int id,
                                             std::vector<robot_zone_id_> &vect)
{
  unsigned int ret = 0;
  for (unsigned int i = 0; i < zones_.size(); i++)
  {
    if (vect.at(i).zone_id == id)
    {
      ret = vect.at(i).robot_id;
      break;
    }
  }
  return ret;
}

//ある地点に対応するゾーンIDを返す
unsigned int regular::point_to_zone_id_(double x, double y)
{
  unsigned int i;
  for (i = 0; i < zones_.size(); i++)
  {
    if (regular::on_area_(x, y, zones_.at(i)))
    {
      break;
    }
  }
  return i;
}

} // agent
} // game
} // ai_server
