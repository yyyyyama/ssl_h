#include <cmath>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/math/constants/constants.hpp>

#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/geometry.h"
#include "ai_server/util/math/to_vector.h"

#include "get_ball.h"
#include "ai_server/model/motion/kick_forward_l.h"
#include "ai_server/model/motion/kick_forward_r.h"
#include <iostream>  //omega表示のため
#include "ai_server/util/math/distance.h"/*distance表示のため*/ 

using boost::math::constants::pi; /*pi=3.14*/

namespace ai_server {
namespace game {
namespace action {

get_ball::get_ball(context& ctx, unsigned int id, const Eigen::Vector2d& target)//targetは二次元ベクトル
    : base(ctx, id),
      state_(running_state::move),//関数の状況を表す。moveで動いている状態,stopで止まっている状態
      target_(target),
      kick_margin_(200),//ロボットがボールを蹴れるかどうかの判断　ボールとロボットの距離 
      manual_kick_flag_({model::command::kick_type_t::line, 45}),
      auto_kick_pow_(45, 100),
      kick_manually_(true),
      allow_(20) {}    // org 10　許容誤差

get_ball::get_ball(context& ctx, unsigned int id)
    : get_ball(ctx, id, Eigen::Vector2d(ctx.world.field().x_max(), 0.0)) {}//敵ゴールの中央位置

get_ball::running_state get_ball::state() const {
  return state_;
}

void get_ball::set_target(double x, double y) {//get ballの位置を決める。何も設定しないと敵ゴール中央になる
  target_ = Eigen::Vector2d{x, y};
}

void get_ball::set_target(const Eigen::Vector2d& t) {//get ballのベクトルを決める
  target_ = t;
}

void get_ball::set_kick_margin(double margin) {
  kick_margin_ = margin;
}

void get_ball::set_allow(double allow) {//許容誤差設定
  allow_ = allow;
}

Eigen::Vector2d get_ball::target() const {
  return target_;
}

void get_ball::kick_automatically() {
  kick_manually_ = false;
}

void get_ball::kick_automatically(int pow) {
  kick_manually_ = false;
  auto_kick_pow_ = {pow, pow};
}

void get_ball::kick_automatically(int line_pow, int chip_pow) {
  kick_manually_ = false;
  auto_kick_pow_ = {line_pow, chip_pow};
}

void get_ball::kick_manually() {
  // kick_manually_ = true;  // org
       kick_manually_ = true;
}

void get_ball::kick_manually(int pow) {
 // kick_manually_                 = true;  // org
  kick_manually_                 = true; 
  std::get<1>(manual_kick_flag_) = pow;
}

void get_ball::kick_manually(model::command::kick_type_t type) {
  // kick_manually_                 = true;  // org
  kick_manually_                 = true; 
  std::get<0>(manual_kick_flag_) = type;
}

void get_ball::kick_manually(const model::command::kick_flag_t& kick) {
 //  kick_manually_    = true;              //org
  kick_manually_    = true; 
  manual_kick_flag_ = kick;
}
//0719ここから
model::command get_ball::execute() {
  model::command command{};
  const auto our_robots   = model::our_robots(world(), team_color());
  const auto enemy_robots = model::enemy_robots(world(), team_color());
  if (!our_robots.count(id_)) return command;
  const auto& robot               = our_robots.at(id_);
  const Eigen::Vector2d robot_pos = util::math::position(robot);
  const Eigen::Vector2d ball_pos  = util::math::position(world().ball());
  const Eigen::Vector2d ball_vel  = util::math::velocity(world().ball());

  if ((ball_pos - target_).norm() < allow_) {//ボールとターゲットの位置＜許容誤差
    state_ = running_state::finished;
    return command;
  }

  // ロボットとボールが十分近づいたと判定する距離
  //constexpr double robot_rad = 125;  // m org
  constexpr double robot_rad = 150;  
  // ドリブルバーの回転速度
  constexpr int dribble_value = 0;
  // ロボットとボールの距離
  const double dist_b_to_r = (robot_pos - ball_pos).norm();
  // ロボットと目標の距離
  const double dist_r_to_t = (robot_pos - target_).norm();

  // ボールを持っているか 説明１
  const bool have_ball =         //bool型なので0 or 1(false or true)
      dist_b_to_r < robot_rad && //ロボットとボールが十分に近づいているかどうか
      std::abs(util::math::wrap_to_pi(
          robot.theta() - std::atan2(ball_pos.y() - robot_pos.y(),
                                     ball_pos.x() - robot_pos.x()))) < pi<double>() / 3.0;  // org 6.0
  
  std::cout << "have_ball " <<  have_ball << "\n";  //mw

  // ロボットの目標角度
  const double theta = std::atan2(target_.y() - robot_pos.y(), target_.x() - robot_pos.x());//目標角度を正面(基準)にする
  std::cout << "target_:" << target_ << "\n"; 

  //test
  auto r_b_dis = util::math::distance(ball_pos,robot_pos);
  std::cout << "r_b_dis : " << r_b_dis << "\n";

  // 速度指令
  switch (state_) {
    // ボールを蹴る
    case running_state::dribble: {
      if (!have_ball) state_ = running_state::move;//have_ballがfalseという条件
      std::cout << "state_ : " << state_ << "\n";
      const Eigen::Vector2d vel = 2.0 * (target_ - robot_pos);
      command.set_velocity(vel);
      command.set_angle(theta);
      break;
    }

    // 移動　
        default: {
      if (have_ball){  // mw 進行方向角度ーの場合左キック、＋の場合右キック
        state_ = running_state::dribble; 
        std::cout << "state_ : " << state_ << "\n";//test
        if(0 > (util::math::wrap_to_pi(
          robot.theta() - std::atan2(ball_pos.y() - robot_pos.y(),
                                     ball_pos.x() - robot_pos.x()))))
             {command.set_motion(std::make_shared<model::motion::kick_forward_l>());}
         else{command.set_motion(std::make_shared<model::motion::kick_forward_r>());}
        
        //mw
      } 
                 
      //説明２ radとposを設定
      const double rad =
          std::abs(util::math::wrap_to_pi(//-180から180に正規化(絶対値)
              robot.theta() - std::atan2(ball_pos.y() - robot_pos.y(),
                                         ball_pos.x() - robot_pos.x()))) < 0.2 * pi<double>()
              ? 90.0   // mw　真
              // ? 90.0  // org 90 150 //10月11日 修正前 150 1801
              : 150.0;//偽
      Eigen::Vector2d pos = ball_pos - rad * (ball_pos - target_).normalized();
      std::cout << "pos : " << pos << "\n";
      //説明３　p1,p2を使った回り込み
      if(std::abs(util::math::wrap_to_pi(
              std::atan2(robot_pos.y() - ball_pos.y(), robot_pos.x() - ball_pos.x()) -
              std::atan2(pos.y() - ball_pos.y(), pos.x() - ball_pos.x()))) >
          0.3 * pi<double>()){
        const auto [p1, p2] = util::math::calc_isosceles_vertexes(robot_pos , ball_pos, rad);
        pos                 = ((p1 - pos).norm() < (p2 - pos).norm()) ? p1 : p2;
        }
      const Eigen::Vector2d vel =
          3.5 * (pos - robot_pos) + (have_ball ? Eigen::Vector2d::Zero() : ball_vel);
      command.set_velocity(vel);
      command.set_angle(theta);
    }
  }

    // キック・ドリブル 説明4
  {
    const Eigen::Vector2d tmp = robot_pos + dist_r_to_t * (Eigen::Rotation2Dd(robot.theta()) *
                                                           Eigen::Vector2d::UnitX());
    const boost::geometry::model::segment<Eigen::Vector2d> line =
        boost::geometry::model::segment(robot_pos, tmp);
    if (dist_b_to_r < robot_rad && boost::geometry::distance(line, target_) < kick_margin_)
      kick(robot_pos, enemy_robots, command);
    if (dist_b_to_r < robot_rad) command.set_dribble(dribble_value);
  } 
  return command;
}

void get_ball::kick(const Eigen::Vector2d& robot_pos,
                    const std::unordered_map<unsigned int, model::robot>& enemy_robots,
                    model::command& command) {
  if (kick_manually_) {
    command.set_kick_flag(manual_kick_flag_);
    return;
  }

  /*
  // チップが有効そうな距離
  const double radius = std::min((robot_pos - target_).norm(), 1000.0);
  //自分からradiusの位置と自分で三角形を作り,間に敵が1つでもあったらチップにする
  const bool flag = std::any_of(
      enemy_robots.cbegin(), enemy_robots.cend(), [this, &robot_pos, radius](const auto& it) {
        const Eigen::Vector2d ene_pos = util::math::position(it.second);
        return (ene_pos - robot_pos).norm() < radius &&
               std::abs(util::math::wrap_to_pi(std::abs(
                   std::atan2(ene_pos.y() - robot_pos.y(), ene_pos.x() - robot_pos.x()) -
                   std::atan2(target_.y() - robot_pos.y(), target_.x() - robot_pos.x())))) <
                   1.0;
      });
  if (flag) {
    command.set_kick_flag({model::command::kick_type_t::chip, std::get<1>(auto_kick_pow_)});
  } else {
    command.set_kick_flag({model::command::kick_type_t::line, std::get<0>(auto_kick_pow_)});
  }   */
}

bool get_ball::finished() const {
  return state_ == running_state::finished;
}

} // namespace action
} // namespace game
} // namespace ai_server