//障害物回避　回り込みプログラム　RightAttckのみ有効Z
#include <cmath>

#include <boost/geometry/geometry.hpp> //calc
#include <boost/geometry/geometries/segment.hpp>
#include <boost/math/constants/constants.hpp>

#include "ai_server/util/math/angle.h" //calc
#include "ai_server/util/math/geometry.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/util/math/distance.h"

#include "rooth.h"
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

rooth::rooth(context& ctx, unsigned int id) : base(ctx, id), move_2walk_(ctx, id) {}
bool rooth::finished() const {
  return false;
}
model::command rooth::execute() {
  model::command command{};
  const auto our_robots   = model::our_robots(world(), team_color());
  const auto enemy_robots = model::enemy_robots(world(), team_color());

  if (!our_robots.count(id_)) return command;
  const auto robot     = our_robots.at(id_);
  const auto ene_robot = enemy_robots.at(id_);

  const auto robot_pos = util::math::position(robot);

  const auto ball_pos = util::math::position(world().ball());
  const auto mergin_r = 15.0 + 90.0 + 15.0; //回り込みマージン
  const auto mergin_t = 180.0 + 90.0;       //回避マージン

    Eigen::Vector2d ene_robot_pos(0, 0);
  if (!enemy_robots.empty()) {
    const auto itr = std::min_element(enemy_robots.cbegin(), enemy_robots.cend(),
                                      [&ball_pos](const auto& a, const auto& b) {
                                        return util::math::distance(a.second, ball_pos) <
                                               util::math::distance(b.second, ball_pos);
                                      });
    ene_robot_pos = util::math::position(itr->second);
  }

  const Eigen::Vector2d ene_goal_pos(world().field().x_min(), 0.0);
  double eg_theta = util::math::direction(ene_goal_pos, ball_pos); //敵ゴール角度
  double pai      = 3.14;

  auto e_goal_ball_x = abs(util::math::distance(ene_goal_pos, ball_pos)
                              *cos(util::math::direction(ball_pos,ene_goal_pos)));
  auto e_goal_robo_x = abs(util::math::distance(ene_goal_pos, robot_pos)
                              *cos(util::math::direction(robot_pos,ene_goal_pos)));
  
  Eigen::Vector2d p1, p2, leftP, rightP, target0;

  /// @brief ある直線の始点と終点が与えられたとき,終点から左右に任意の長さ分ずらした2点を返す
  /// @param apex ある直線についての始点
  /// @param middle_base ある直線についての終点
  /// @param shift 終点からずらしたい長さ
  std::tie(p1, p2) = util::math::calc_isosceles_vertexes(ene_goal_pos, ball_pos, mergin_r);

  /// @brief   ある円と，任意の点から引いたその円への接線があるとき，その接点を求めて返す
  /// @param p      円の外側にある任意の点
  /// @param center 円の中心
  /// @param r      円の半径
  std::tie(leftP, rightP) = util::math::contact_points(robot_pos, ene_robot_pos, mergin_t);

  auto bp_x     = ball_pos.x();
  auto bp_y     = ball_pos.y();
  auto rp_x     = robot_pos.x();
  auto rp_y     = robot_pos.y();
  auto e_rp_x   = ene_robot_pos.x(); // x to y no +- ga gyaku ?
  auto e_rp_y   = ene_robot_pos.y();
  auto e_goal_x = ene_goal_pos.x();

  // auto target0_pos = ball_pos;                   //posの設定？
  // auto target0.x() = bp_x-mergin_r*cos(eg_theta);  //ここを修正？
  // auto target0.y() = bp_y-mergin_r*sin(eg_theta);
  // target0_pos = {target0.x(),target0.y()};
  Eigen::Vector2d target0_pos = ball_pos + mergin_r * (ball_pos - ene_goal_pos).normalized();

  double omega = util::math::direction_from(
      std::atan2(target0.y() - robot_pos.y(), target0.x() - robot_pos.x()), robot.theta());
  double e_omega = util::math::direction_from(target0_pos, robot_pos, ene_robot_pos, robot_pos);
  constexpr double rot_th = 0.5;

   if(e_goal_ball_x >= e_goal_robo_x){  //回り込み判別　何故敵ゴールは全て上なのか？
  　
    if ((util::math::distance(p1, robot_pos)) <= (util::math::distance(p2, robot_pos))) {
  
      target0 = p1;
    } else { // P2が下
      target0 = p2;
    }

       omega = util::math::direction_from(util::math::direction(target0, robot_pos),robot.theta());
       move_2walk_.set_omega(omega);
       return move_2walk_.execute();
    
  } 
  
  else if(mergin_r <= util::math::distance(ball_pos, robot_pos)){  
    //ボールとロボットの距離がマージンより大きい時のターゲットはボールよりマージン分大きい  
    target0 = ball_pos + mergin_r * (ball_pos - ene_goal_pos).normalized();
    omega = util::math::direction_from(util::math::direction(target0, robot_pos),robot.theta());

    std::cout << "target0 " << target0 << "\n";    
  
     move_2walk_.set_omega(omega);
     return move_2walk_.execute();


  }else{omega = util::math::direction_from(util::math::direction(ball_pos, robot_pos),robot.theta());
　　　　　　 //ボールとロボットの距離がマージンより小さい時のターゲットはボール 
         move_2walk_.set_omega(omega);
         return move_2walk_.execute();
       }  

  if (mergin_t > util::math::distance(ene_robot_pos, robot_pos) *
                     sin(e_omega)) { //敵ロボットとの距離がマージン以内か
    if (e_omega > -1.57 && e_omega < 1.57) { //前方にいるか
      if ((util::math::distance(leftP, robot_pos)) <
          (util::math::distance(rightP, robot_pos))) {
        target0 = leftP;
      } else {
        target0 = rightP;
      }

      // std::cout << "LeftP " << leftP << "\n";
      // std::cout << "rightP " << rightP << "\n";

      omega = util::math::direction_from(util::math::direction(target0, robot_pos),robot.theta());

    }

  move_2walk_.set_omega(omega);
  return move_2walk_.execute();

  }

                      //omega表示のため

  // std::cout << "eg_theta " <<  eg_theta << "\n";
  // std::cout << "robot_pos " <<  robot_pos  << "\n";
  // std::cout << "e_robot_y " <<  e_rp_y  << "\n";
  // std::cout << "leftP "     <<  leftP  <<  "\n";
  // std::cout << "rightP "    <<  rightP  << "\n";
  // std::cout << "tgt0_x "   <<  target0.x()  << "\n";
  // std::cout << "tgt0_y "   <<  target0.y()  << "\n";
  // std::cout << "omega "    <<  omega  << "\n";
  // std::cout << "e_omega "    <<  e_omega  << "\n";

  return command;
}

} // namespace action
} // namespace game
} // namespace ai_server