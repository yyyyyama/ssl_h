#include <unordered_map>
#include <cmath>

#include "regular.h"
#include "ai_server/model/robot.h"

namespace ai_server {
namespace game {
namespace agent {

regular::regular(const model::world& world, bool is_yellow,
                 const std::vector<unsigned int>& ids)
    : base(world, is_yellow), ids_(ids) {}

std::vector<std::shared_ptr<action::base>> regular::execute() {
  std::vector<std::shared_ptr<action::base>> actions;

  const auto ball = world_.ball();
  // const auto these_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();

  // std::vector<unsigned int> this_ids;  //味方ロボットのID
  std::vector<unsigned int> that_ids;  //敵ロボットのID
  std::vector<unsigned int> added_ids; //動作が既に割当てられているロボットのID
  unsigned int candidate_id = ids_.at(0); //ある動作を割り当てられる候補のロボットのID

  /*double shortest_d = std::hypot(world_.field().x_max(), world_.field().y_max()) +
                      10000.0; //距離で割当するときに使用*/

  //敵ロボットのIDを登録
  for (auto that_rob : those_robots) {
    that_ids.push_back(that_rob.first);
  }

  //ボールを追いかけるロボットの登録
  candidate_id = nearest_robot_id(ball.x(), ball.y());

  // ballを追いかけるActionを設定
  // chase_ball = regular::make_action(candidate_id);
  // actions.push_back(chase_ball);
  auto move = std::make_shared<action::move>(world_, is_yellow_, candidate_id); // for debug
  move->move_to(ball.x(), ball.y(), 0.0);                                       // for debug
  actions.push_back(move);                                                      // for debug
  added_ids.push_back(candidate_id);
  std::printf("\x1b[46mchase ball ID: %d\x1b[49m\n\n", candidate_id); // for debug

  //マーキング対象のロボットを、優先度の高い順に登録する
  std::vector<unsigned int> that_soat_ids;
  std::vector<unsigned int>::iterator that_ids_tag = that_ids.end() - 1;
  unsigned int that_id;

  while (!that_ids.empty()) {
    that_id = *that_ids_tag;
    for (auto id : that_ids) {
      if (std::abs(those_robots.at(id).x() - world_.field().x_min()) < shortest_d) {
        shortest_d = std::abs(those_robots.at(that_id).x() - world_.field().x_min());
      }
    }
    that_ids.erase(that_ids_tag);
    that_ids_tag--;
  }
  for (auto id : that_ids) {
    //  if
  }

  //マーキングを担当するロボットを割り当て
  // shortest_d = for (auto id : ids_) {}

  return actions;
  std::printf("-----     One Loop     -----\n\n"); // debug
}

unsigned int regular::nearest_robot_id(double target_x, double target_y) {
  const double v_stop_max = 50.0; //ロボットが静止していると判定する上限値
  double smallest_theta   = std::asin(
      3.0 /
      7.0); //ロボットの速度ベクトルと、ロボットから見たボールの方向のズレの最大許容値を指定(0<=smallest_theta<=π/2)

  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  double shortest_d     = std::hypot(world_.field().x_max(), world_.field().y_max()) + 10000.0;

  double robot_target_d;
  double ret_id = ids_.at(0);

  for (auto id : ids_) {
    robot_target_d =
        std::hypot(target_x - ids_robots.at(id).x(), target_y - ids_robots.at(id).y());
    std::printf("ID:%d--%5.0f[mm]", id, robot_target_d);
    if (robot_target_d < shortest_d) {
      // Targetとの距離

      if (std::hypot(ids_robots.at(id).vx(), ids_robots.at(id).vy()) < v_stop_max) {
        //ロボットが静止している時
        shortest_d = robot_target_d;
        ret_id     = id;
        std::printf("<--IF 1"); // debug
      } else if (std::abs(std::atan2(ids_robots.at(id).vy(), ids_robots.at(id).vx()) -
                          std::atan2(target_y - ids_robots.at(id).y(),
                                     target_x - ids_robots.at(id).x())) < smallest_theta &&
                 ids_robots.at(id).vx() * (target_x - ids_robots.at(id).x()) +
                         ids_robots.at(id).vy() * (target_y - ids_robots.at(id).y()) >=
                     0.0) {
        //ロボットの速度ベクトルの角度がTargetの方向に近い時
        shortest_d     = robot_target_d;
        ret_id         = id;
        smallest_theta = std::abs(
            std::atan2(ids_robots.at(id).vy(), ids_robots.at(id).vx()) -
            std::atan2(target_y - ids_robots.at(id).y(), target_x - ids_robots.at(id).x()));
        std::printf("<--IF 2"); // debug
      }
    }
    std::printf("\n"); // debug
  }

  return ret_id;
}

} // agent
} // game
} // ai_server
