#include <cmath>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/assign/list_of.hpp>

#include "ai_server/game/action/get_ball.h"
#include "ai_server/util/math.h"
#include "ai_server/util/math/to_vector.h"

namespace ai_server {
namespace game {
namespace action {

get_ball::get_ball(const model::world& world, bool is_yellow, unsigned int id)
    : base(world, is_yellow, id), flag_(false), target_(Eigen::Vector2d::Zero()) {}

void get_ball::set_target(double x, double y) {
  target_ = Eigen::Vector2d{x, y};
}
void get_ball::set_target(Eigen::Vector2d target) {
  target_ = target;
}

model::command get_ball::execute() {
  namespace bg  = boost::geometry;
  using point   = bg::model::d2::point_xy<double>;
  using polygon = bg::model::polygon<point>;
  using boost::math::constants::pi;

  model::command command(id_);

  const auto my_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if (!my_robots.count(id_)) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  const auto& my_robot   = my_robots.at(id_);
  const auto robot       = util::math::position(my_robot);
  const auto robot_theta = util::wrap_to_pi(my_robot.theta());
  // const Eigen::Vector2d my{my_robot.x(), my_robot.y()};
  const auto ball_pos = util::math::position(world_.ball());
  const auto ball_vec = util::math::velocity(world_.ball());
  // 0.5秒後のボールの位置
  const auto ball = ball_pos + ball_vec * 0.5;
  // const Eigen::Vector2d ball{world_.ball().x(), world_.ball().y()};

  //目標位置と自分の位置で四角を作る
  polygon poly;
  bg::exterior_ring(poly) = boost::assign::list_of<point>(robot.x(), world_.field().y_min())(
      target_.x(), world_.field().y_min())(target_.x(), world_.field().y_max())(
      robot.x(), world_.field().y_max())(robot.x(), world_.field().y_min());
  const point p(ball.x(), ball.y());

  //ロボットの中心から口までの距離
  const auto radius = 70;
  //移動目標
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  //移動速度
  auto speed = 0.0;

  if (!bg::disjoint(p, poly)) {
    //回り込む
    //キックフラグを立てて置く
    command.set_kick_flag(model::command::kick_flag_t{model::command::kick_type_t::none, 0});
    command.set_dribble(9);
    const auto ratio = radius / ((ball_pos - ball).norm() + radius); //ボール - 目標位置の比
    position         = (-ratio * ball_pos + 1 * ball) / (1 - ratio);
    speed            = ball_vec.norm() * 2.0 < 1000.0 ? 1000.0 : ball_vec.norm() * 2.0;
    {
      //移動経路に他のものがあったらよける
      //判定の幅
      const auto mergin1 = 300.0;
      const auto mergin2 = 600.0;
      //軌道の三角形
      polygon poly;
      bg::exterior_ring(poly) = boost::assign::list_of<point>(robot.x(), robot.y())(
          std::get<0>(util::move(robot, position, mergin1)).x(),
          std::get<0>(util::move(robot, position, mergin1)).y())(
          std::get<1>(util::move(robot, position, mergin1)).x(),
          std::get<1>(util::move(robot, position, mergin1)).y())(robot.x(), robot.y());

      const point p(ball.x(), ball.y());
      if (!bg::disjoint(p, poly)) {
        const auto tmp  = util::move(robot, ball, mergin2);
        const auto tmp1 = std::get<0>(tmp);
        const auto tmp2 = std::get<1>(tmp);
        position        = std::abs(tmp1.y()) < std::abs(tmp2.y()) ? tmp1 : tmp2;
      }
    }
  } else {
    //目標に向かって蹴る
    //キックフラグを立てて置く
    command.set_kick_flag(model::command::kick_flag_t{model::command::kick_type_t::chip,10});
    command.set_dribble(0);
    const auto ratio = radius / ((target_ - ball).norm() + radius); //ボール - 目標位置の比
    position         = (-ratio * target_ + 1 * ball) / (1 - ratio);
    speed            = ball_vec.norm() * 2.0 < 1000.0 ? 1000.0 : ball_vec.norm() * 2.0;
  }
  //向きをボールの方へ
  const auto theta =
      util::wrap_to_pi(std::atan2(position.y() - ball.y(), position.x() - ball.x()));
  const auto omega = theta - robot_theta;

  //目標位置からベクトルへ
  const auto velocity = (position - robot).normalized() * speed;
  command.set_velocity({velocity.x(), velocity.y(), omega});
  return command;
}

bool get_ball::finished() const {
  return flag_;
}

} // namespace ai_server
} // namespace game
} // namespace action
