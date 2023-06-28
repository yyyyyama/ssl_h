#include <cmath> //数学的関数
#include "clear.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/model/motion/walk_forward.h"
#include "ai_server/model/motion/turn_left.h"
#include "ai_server/model/motion/turn_right.h"
#include <iostream> /*omega表示のため*/
#include "ai_server/util/math/distance.h"/*distance表示のため*/ 
#include "ai_server/util/math/detail/direction.h" /*direction表示のため*/


double omega;

namespace ai_server::game::action{
    clear::clear(context& ctx,unsigned int id):base(ctx,id){}
    //IDはロボットの番号（IDを入れるとそのロボットだけ動く）

bool clear::finished()const{
    // 終了条件は設定しないため常にfalse
    return false;
}

model::command clear::execute() {
 model::command command{};
 // 自チームのロボットの情報を取得
 const auto our_robots = model::our_robots(world(), team_color());
 // 自分の情報がなければ終了
 if (!our_robots.count(id_)) return command;
 // 自分の情報を取得
 const auto robot = our_robots.at(id_);
 // 自分とボールの位置を取得
 const auto robot_pos = util::math::position(robot);
 const auto ball_pos = util::math::position(world().ball());
 // 前進
 command.set_motion(std::make_shared<model::motion::walk_forward>());
 // 向きが合っていなければ回転(前進のモーションはキャンセルされる)aaaa
 constexpr double rot_th = 0.5;
  //auto omega = util::math::direction_from(util::math::direction(target0,robot_pos),robot.theta());
 auto omega = util::math::direction_from(util::math::direction(ball_pos,robot_pos),robot.theta());
  if (rot_th < omega ){
 command.set_motion(std::make_shared<model::motion::turn_left>());
 } else if (omega < -rot_th) {
 command.set_motion(std::make_shared<model::motion::turn_right>());
 }
 std::cout << "omega: " << omega << "\n";/*omega出力*/
 auto dir = util::math::direction(ball_pos,robot_pos);
 std::cout << "direction: " << dir << "\n";/*direction test*/
 auto dis = util::math::distance(ball_pos,robot_pos);
 std::cout << "distance: " << dis << "\n";/*distance test*/
 auto dir_from = util::math::direction_from(util::math::direction(ball_pos,robot_pos),robot.theta());
 std::cout << "direction_from: " << dir_from << "\n";/*direction_from test*/
return command;
}

} // namespace ai_server::game::action
