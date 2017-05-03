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
  const double v_stop_max = 50.0; //ロボットが静止していると判定する上限値

  std::vector<std::shared_ptr<action::base>> actions;
  const auto ball         = world_.ball();
  const auto these_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto those_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  //   std::vector<model::robot> this_robots;
  std::vector<model::robot> that_ids;  //敵ロボットのID
  std::vector<unsigned int> added_ids; //動作が既に割当てられているロボットのID
  unsigned int candidate_id = ids_.at(0); //ある動作を割り当てられる候補のロボットのID

  //味方ロボットの登録
  //  for(auto id : ids_) {
  //	 this_robots.push_back(these_robots.at(id));
  //  }

  //敵ロボットのIDを登録
  for (auto that_rob : those_robots) {
    that_ids.push_back(that_rob.first);
  }

  //ボールを追いかけるロボットの登録
  double shortest_d = std::hypot(world_.field().x_max(), world_.field().y_max()) + 10000.0;

  double smallest_theta = std::asin(
     4.0/7.0); //ロボットの速度ベクトルと、ロボットから見たボールの方向のズレの最大許容値を指定する。
  //                                            (ただし、0<=smallest_theta<=π/2)
  double robot_ball_d;
  for (auto id : ids_) {
    robot_ball_d =
        std::hypot(ball.x() - these_robots.at(id).x(), ball.y() - these_robots.at(id).y());
    std::printf("ID:%d--%.0f", id, robot_ball_d);
    if (robot_ball_d < shortest_d) {
      //ボールとの距離

      if (std::hypot(these_robots.at(id).vx(), these_robots.at(id).vy()) < v_stop_max) {
        //ロボットが静止している時
        shortest_d   = robot_ball_d;
        candidate_id = id;
        std::printf("<--IF 1"); // debug
      } else if (std::abs(std::atan2(these_robots.at(id).vy(), these_robots.at(id).vx()) -
                          std::atan2(ball.y() - these_robots.at(id).y(),
                                     ball.x() - these_robots.at(id).x())) < smallest_theta &&
                 these_robots.at(id).vx() * (ball.x() - these_robots.at(id).x()) +
                         these_robots.at(id).vy() * (ball.y() - these_robots.at(id).y()) >=
                     0.0) {
        //ロボットの速度ベクトルの角度がボールの方向に近い時
        shortest_d     = robot_ball_d;
        candidate_id   = id;
        smallest_theta = std::abs(
            std::atan2(these_robots.at(id).vy(), these_robots.at(id).vx()) -
            std::atan2(ball.y() - these_robots.at(id).y(), ball.x() - these_robots.at(id).x()));
        std::printf("<--IF 2"); // debug
    }
}
    std::printf("\n"); // debug
}

  // ballを追いかけるActionを設定
  // chase_ball = regular::make_action(candidate_id);
  // actions.push_back(chase_ball);
  auto move = std::make_shared<action::move>(world_, is_yellow_, candidate_id); // for debug
  move->move_to(ball.x(), ball.y(), 0.0);                                       // for debug
  actions.push_back(move);                                                      // for debug
  added_ids.push_back(candidate_id);
  std::printf("\x1b[46mchase ball ID: %d\x1b[49m\n\n", candidate_id); // for debug
  
  
  //マーキングするロボットの登録
  std::vector<unsigned int> that_soat_ids;
  for (auto id : that_ids) {
  }

  return actions;
}

} // agent
} // game
} // ai_server
