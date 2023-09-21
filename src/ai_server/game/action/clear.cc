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
//0628追加
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/math/constants/constants.hpp>
#include "ai_server/util/math/geometry.h"

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
 //敵ゴールの位置を取得
 const Eigen::Vector2d ene_goal_pos (world().field().x_min(),0.0);
 const auto target_0 = ene_goal_pos;
 auto rad = 90.0;
 // 前進
 command.set_motion(std::make_shared<model::motion::walk_forward>());
 // 向きが合っていなければ回転(前進のモーションはキャンセルされる)
 constexpr double rot_th = 0.3;

//robotとballの距離と角度
auto r_b_dis = util::math::distance(ball_pos,robot_pos);
auto r_b_deg = util::math::direction(ball_pos,robot_pos);

 Eigen::Vector2d pos = ball_pos - rad * (ball_pos - target_0).normalized();

auto pos_b_dis = util::math::distance(ball_pos,pos);
auto pos_b_deg = util::math::direction(ball_pos,pos);

auto pos_r_dis = util::math::distance(pos,robot_pos);
auto pos_r_deg = util::math::direction(robot_pos,pos);

auto target_0_r_dis = util::math::distance(robot_pos,target_0);
auto target_0_r_deg = util::math::direction(robot_pos,target_0);

 auto omega = util::math::direction_from(util::math::direction(pos,robot_pos),robot.theta());

if (rot_th < omega ){
auto target_0_b_dis = util::math::distance(ball_pos,target_0);
auto target_0_b_deg = util::math::direction(ball_pos,target_0);
 command.set_motion(std::make_shared<model::motion::walk_forward>());
 // 向きが合っていなければ回転(前進のモーションはキャンセルされる)
 }

Eigen::Vector2d p1, p2, leftP, rightP, target0;
const auto mergin_r = 90.0;
std::tie(p1, p2) = util::math::calc_isosceles_vertexes(robot_pos , ball_pos, mergin_r);

  //auto omega = util::math::direction_from(util::math::direction(target0,robot_pos),robot.theta());

//p1,p2を使った回り込みに使う
auto r_p1_dis = util::math::distance(robot_pos,p1);
auto r_p2_dis = util::math::distance(robot_pos,p2);
//p1,p2を使った回り込み
if (r_b_dis < pos_r_dis){
    if (r_p1_dis < r_p2_dis){
    pos = p1;
  }else{
    pos = p2;
  }
 }

  if (rot_th < omega ){
 command.set_motion(std::make_shared<model::motion::turn_left>());
 } else if (omega < -rot_th) {
 command.set_motion(std::make_shared<model::motion::turn_right>());
 }

//９月１０日実験
auto pos_p1_dis = util::math::distance(pos,p1);
auto pos_p2_dis = util::math::distance(pos,p2);
auto p1_p2_dis = util::math::distance(p1,p2);

 //std::cout << "omega: " << omega << "\n";/*omega出力*/
 //auto dir_from = util::math::direction_from(util::math::direction(ball_pos,robot_pos),robot.theta());
 //std::cout << "direction_from: " << dir_from << "\n";/*direction_from test*/
 std::cout << "r_b_dis: " << r_b_dis << "\n"; 
 //std::cout << "r_b_deg: " << r_b_deg << "\n";
 std::cout << "pos_b_dis: " << pos_b_dis << "\n";
 //std::cout << "pos_b_deg: " << pos_b_deg << "\n";
 std::cout << "ball_pos: " << ball_pos << "\n";
 std::cout << "pos: " << pos << "\n";/*pos表示*/
 std::cout << "pos_r_dis: " << pos_r_dis << "\n";
 //std::cout << "target_0: " << target_0 << "\n";
 std::cout << "p1: " << p1 << "\n";
 std::cout << "p2: " << p2 << "\n";
 std::cout << "pos_p1_dis: " << pos_p1_dis << "\n";
 std::cout << "pos_p2_dis: " << pos_p2_dis << "\n";
 std::cout << "p1_p2_dis: " << p1_p2_dis << "\n";
return command;
}
}
// namespace ai_server::game::action



