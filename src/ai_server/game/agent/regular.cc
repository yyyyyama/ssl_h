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

  std::vector<unsigned int> this_unadded_ids; //まだ動作の割り当てられていない味方ロボットのID
  std::vector<unsigned int> those_ids; //敵ロボットのID
  std::vector<unsigned int> added_ids; //動作が既に割当てられているロボットのID
  unsigned int candidate_id = ids_.at(0); //ある動作を割り当てられる候補のロボットのID

  this_unadded_ids = ids_;

  //敵ロボットのIDを登録
  for (auto that_rob : those_robots) {
    those_ids.push_back(that_rob.first);
  }

  //ボールを追いかけるロボットの登録
  candidate_id = nearest_robot_id(ball.x(), ball.y(), this_unadded_ids);

  // ballを追いかけるActionを設定
  // chase_ball = regular::make_action(candidate_id);
  // actions.push_back(chase_ball);

  // <debug>
  auto move = std::make_shared<action::move>(world_, is_yellow_, candidate_id);
  move->move_to(ball.x(), ball.y(), 0.0);
  actions.push_back(move);
  added_ids.push_back(candidate_id);
  std::printf("\x1b[46mchase ball ID: %d\x1b[49m\n\n", candidate_id);
  // </debug>

  //対象敵のIDと重要度を登録
  std::vector<regular::that_robot_importance_> that_importance_list;
  for (auto id : those_ids) {
    that_importance_list.push_back({id, -1.0 * those_robots.at(id).x()});
  }

  //重要度の高い順にソート
  std::sort(that_importance_list.begin(), that_importance_list.end());

  // <debug>
  for (auto id : that_importance_list) {
    std::printf("ID: %d  \x1b[36m重要度: %.0f\x1b[39m\n", id.id, id.importance);
  }
  // </debug>

  //マーキングを担当するロボットを割り当て

  // shortest_d = for (auto id : ids_) {}

  return actions;
  std::printf("\n-----     One Loop     -----\n\n"); // debug
}

unsigned int regular::nearest_robot_id(double target_x, double target_y,
                                       std::vector<unsigned int>& can_ids) {
  const double v_stop_max = 50.0; //ロボットが静止しているとするVの上限
  double smallest_theta   = std::asin(
      3.0 /
      7.0); //ロボットのVベクトルと、ロボットから見たボールの方向のズレの最大許容値を指定(0<=smallest_theta<=π/2)

  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  double shortest_d     = std::hypot(world_.field().x_max(), world_.field().y_max()) + 10000.0;

  double robot_target_d;
  double ret_id = ids_.at(0);

  for (auto id : can_ids) {
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
