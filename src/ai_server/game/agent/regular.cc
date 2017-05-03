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

  const auto ball         = world_.ball();
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();

  std::vector<unsigned int> this_unadded_ids; //まだ動作の割り当てられていない味方ロボットのID
  std::vector<unsigned int> those_ids; //敵ロボットのID
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
  move->move_to(ball.x() + 20, ball.y() + 20, 0.0);
  actions.push_back(move);
  std::printf("\x1b[46mchase ball ID: %d\x1b[49m\n\n", candidate_id);
  // </debug>

  //対象敵のIDと重要度を登録し、重要度の高い順にソート
  std::vector<regular::that_robot_importance_> that_importance_list;
  for (auto id : those_ids) {
    that_importance_list.push_back({id, -1.0 * those_robots.at(id).x()});
  }
  std::sort(that_importance_list.begin(), that_importance_list.end());

  //マーキングを担当するロボットを割り当て
  for (auto that_robot : that_importance_list) {
    if (!this_unadded_ids.empty()) {
      candidate_id = nearest_robot_id(those_robots.at(that_robot.id).x(),
                                      those_robots.at(that_robot.id).x(), this_unadded_ids);
      marking_ = std::make_shared<action::marking>(world_, is_yellow_, candidate_id);
      marking_->mark_robot(that_robot.id);
      marking_->set_mode(that_robot.importance > -0.0
                             ? action::marking::mark_mode::kick_block
                             : action::marking::mark_mode::shoot_block);
      marking_->set_radius(200.0);
      actions.push_back(marking_);

      std::printf("\x1b[44m'%d'\x1b[49m marks to \x1b[44m'%d'\x1b[49m\n\n", candidate_id,
                  that_robot.id); // debug
    } else {
      break;
    }
  }

  std::printf(
      "\n\x1b[32m***** ***** ***** ***** One Loop ***** ***** ***** *****\x1b[39m\n\n"); // debug
  return actions;
}

unsigned int regular::nearest_robot_id(double target_x, double target_y,
                                       std::vector<unsigned int>& can_ids) {
  const double v_stop_max = 10000.0; //ロボットが静止しているとするVの上限
  double smallest_theta   = std::asin(
      1.0); //ロボットのVベクトルと、ロボットから見たボールの方向のズレの最大許容値を指定(0<=smallest_theta<=π/2)

  double ret_id         = ids_.at(0);
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  double robot_target_d;
  double shortest_d = std::hypot(world_.field().x_max(), world_.field().y_max()) + 50.0;
  auto del_id_itr   = can_ids.begin();

  for (auto id_itr = can_ids.begin(); id_itr < can_ids.end(); ++id_itr) {
    robot_target_d = std::hypot(target_x - ids_robots.at(*id_itr).x(),
                                target_y - ids_robots.at(*id_itr).y());
    std::printf("ID:%d--%5.0f[mm]", *id_itr, robot_target_d);
    if (robot_target_d < shortest_d) {
      // Targetとの距離

      if (std::hypot(ids_robots.at(*id_itr).vx(), ids_robots.at(*id_itr).vy()) < v_stop_max) {
        //ロボットが静止している時
        shortest_d = robot_target_d;
        ret_id     = *id_itr;
        del_id_itr = id_itr;
        std::printf("<--IF 1"); // debug
      } else if (std::abs(std::atan2(ids_robots.at(*id_itr).vy(), ids_robots.at(*id_itr).vx()) -
                          std::atan2(target_y - ids_robots.at(*id_itr).y(),
                                     target_x - ids_robots.at(*id_itr).x())) < smallest_theta &&
                 ids_robots.at(*id_itr).vx() * (target_x - ids_robots.at(*id_itr).x()) +
                         ids_robots.at(*id_itr).vy() *
                             (target_y - ids_robots.at(*id_itr).y()) >=
                     0.0) {
        //ロボットの速度ベクトルの角度がTargetの方向に近い時
        shortest_d = robot_target_d;
        ret_id     = *id_itr;
        del_id_itr = id_itr;
        smallest_theta =
            std::abs(std::atan2(ids_robots.at(*id_itr).vy(), ids_robots.at(*id_itr).vx()) -
                     std::atan2(target_y - ids_robots.at(*id_itr).y(),
                                target_x - ids_robots.at(*id_itr).x()));
        std::printf("<--IF 2"); // debug
      }
    }
    std::printf("\n"); // debug
  }
  can_ids.erase(del_id_itr);
  return ret_id;
}

} // agent
} // game
} // ai_server
