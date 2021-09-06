//回り込みプログラム　RightAttckのみ有効Z
#include <cmath>

#include <boost/geometry/geometry.hpp> //calc
#include <boost/geometry/geometries/segment.hpp>
#include <boost/math/constants/constants.hpp>

#include "ai_server/util/math/angle.h" //calc
#include "ai_server/util/math/geometry.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/util/math/distance.h"

#include "clear.h"
#include "ai_server/model/motion/walk_forward.h"
#include "ai_server/model/motion/turn_left.h"
#include "ai_server/model/motion/turn_right.h"
#include "ai_server/model/motion/walk_left.h"
#include "ai_server/model/motion/walk_right.h"
#include "ai_server/model/motion/stop.h"

#include <iostream>  //omega表示のため
using namespace std; // omega表示のため

using boost::math::constants::pi;

namespace ai_server {
namespace game {
namespace action {

clear::clear(context& ctx, unsigned int id) : base(ctx, id),move_2walk_(ctx, id) {}
bool clear::finished() const {
  return false;
}
model::command clear::execute() {
  model::command command{};
  const auto our_robots = model::our_robots(world(), team_color());

  if (!our_robots.count(id_)) return command;
  const auto robot = our_robots.at(id_);
  const auto robot_pos = util::math::position(robot);
  const auto ball_pos  = util::math::position(world().ball());
  const auto mergin_r   = 30.0 + 90.0;

  const Eigen::Vector2d ene_goal_pos(world().field().x_max(), 0.0);
  double eg_theta = util::math::direction(ene_goal_pos, ball_pos);

  Eigen::Vector2d p1, p2;
  std::tie(p1, p2) = util::math::calc_isosceles_vertexes(ene_goal_pos, ball_pos, mergin_r);
  Eigen::Vector2d target0_pos = ball_pos + mergin_r * (ball_pos - ene_goal_pos).normalized();
  
  auto bp_x = ball_pos.x();
  auto bp_y = ball_pos.y();
  auto rp_x = robot_pos.x();
  // auto rp_y = robot_pos.y();

  auto e_goal_x = ene_goal_pos.x();
  auto e_goal_ball_x = abs(util::math::distance(ene_goal_pos, ball_pos)
                              *cos(util::math::direction(ball_pos,ene_goal_pos)));
  auto e_goal_robo_x = abs(util::math::distance(ene_goal_pos, robot_pos)
                              *cos(util::math::direction(robot_pos,ene_goal_pos)));
  // auto e_goal_y = ene_goal_pos.y();
  // auto target0.x() = bp_x-mergin_r*cos(eg_theta);
  // auto target0.y() = bp_y-mergin_r*sin(eg_theta);
  Eigen::Vector2d target0 = ball_pos + mergin_r * (ball_pos - ene_goal_pos).normalized();
  double omega = util::math::direction_from(util::math::direction(target0, robot_pos),robot.theta());
  
  //std::cout << "omega " << omega << "\n";
  //std::cout << "e_goal_ball_x " << e_goal_ball_x << "\n";
  //std::cout << "e_goal_robo_x " << e_goal_robo_x << "\n";

  /*double omega = util::math::direction_from(
      std::atan2(target0.y() - robot_pos.y(), target0.x() - robot_pos.x()), robot.theta());
  */
  constexpr double rot_th = 0.5;

  if(e_goal_ball_x >= e_goal_robo_x){  //回り込み判別　何故敵ゴールは全て上なのか？
  　
    if ((util::math::distance(p1, robot_pos)) <= (util::math::distance(p2, robot_pos))) {
      target0 = p1;} //P1が上
    else {
      target0 = p2;} //P2が下
   
   omega = util::math::direction_from(util::math::direction(target0, robot_pos),robot.theta());
   
   move_2walk_.set_omega(omega);
   return move_2walk_.execute();

  }
  else if(mergin_r <= util::math::distance(ball_pos, robot_pos)){
    target0 = ball_pos + mergin_r * (ball_pos - ene_goal_pos).normalized();
    omega = util::math::direction_from(util::math::direction(target0, robot_pos),robot.theta());

    std::cout << "target0 " << target0 << "\n";    
  
     move_2walk_.set_omega(omega);
     return move_2walk_.execute();


}else{omega = util::math::direction_from(util::math::direction(ball_pos, robot_pos),robot.theta());

       move_2walk_.set_omega(omega);
       return move_2walk_.execute();
     }  

  /*omega表示のため

  std::cout << "eg_theta" << eg_theta << "\n";
  std::cout << "ball_pos_x" << bp_x << "\n";
  std::cout << "ball_pos_y" << bp_y << "\n";

  std::cout << "tgt0_x" << target0.x() << "\n";
  std::cout << "tgt0_y" << target0.y() << "\n";
*/
  return command;
}

} // namespace action
} // namespace game
} // namespace ai_server