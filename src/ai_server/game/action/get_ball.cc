#include <cmath>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/assign/list_of.hpp>

#include "ai_server/game/action/get_ball.h"
#include "ai_server/util/math.h"
#include "ai_server/util/math/geometry.h"
#include "ai_server/util/math/to_vector.h"

namespace ai_server {
namespace game {
namespace action {

get_ball::get_ball(const model::world& world, bool is_yellow, unsigned int id)
    : base(world, is_yellow, id), target_(Eigen::Vector2d::Zero()), flag_(false) {}

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
  const auto robot_vec   = util::math::velocity(my_robot);

  const auto ball_pos = util::math::position(world_.ball());
  const auto ball_vec = util::math::velocity(world_.ball());
  // 2.0秒後のボールの位置
  const Eigen::Vector2d ball = ball_pos + ball_vec * 2.0;

  //移動目標
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};

  if (ball_vec.norm() < 1000.0) {
    //目標に向かって蹴る
    auto radius = 140;
    {
      //目標位置に向いたら蹴る
      const auto theta =
          util::wrap_to_pi(std::atan2(target_.y() - robot.y(), target_.x() - robot.x()));
      if (std::abs(theta - robot_theta) < pi<double>() / 31.0) {
        command.set_kick_flag(
            model::command::kick_flag_t{model::command::kick_type_t::line, 255});
        command.set_dribble(0);
      } else {
        command.set_kick_flag(
            model::command::kick_flag_t{model::command::kick_type_t::none, 0});
        command.set_dribble(9);
      }
      radius = 70;
    }
    //移動目標はボールのちょい後ろ
    {
      //ロボットの中心から口までの距離
      const auto ratio =
          radius / ((target_ - ball_pos).norm() + radius); //ボール - 目標位置の比
      position = (-ratio * target_ + 1 * ball_pos) / (1 - ratio);
    }
    {
      //目標位置と自分の位置で四角を作る
      const auto mergin = 70.0;
      polygon poly;
      const auto tmp          = util::math::calc_isosceles_vertexes(robot, position, mergin);
      bg::exterior_ring(poly) = boost::assign::list_of<point>(robot.x(), robot.y())(
          std::get<0>(tmp).x(), std::get<0>(tmp).y())(
          std::get<1>(tmp).x(), std::get<1>(tmp).y())(robot.x(), robot.y());
      const point p(ball_pos.x(), ball_pos.y());
      //間にボールがあったら回り込む
      if (!bg::disjoint(p, poly)) {
        //判定の幅
        const auto mergin = 500.0;
        const auto tmp1 =
            std::get<0>(util::math::calc_isosceles_vertexes(position, ball_pos, mergin));
        const auto tmp2 =
            std::get<1>(util::math::calc_isosceles_vertexes(position, ball_pos, mergin));

        const auto tc = (ball_pos.x() - target_.x()) * (tmp1.y() - ball_pos.y()) +
                        (ball_pos.y() - target_.y()) * (ball_pos.x() - tmp1.x());
        const auto td = (ball_pos.x() - target_.x()) * (tmp2.y() - ball_pos.y()) +
                        (ball_pos.y() - target_.y()) * (ball_pos.x() - tmp2.x());
        const auto tp = (ball_pos.x() - target_.x()) * (robot.y() - ball_pos.y()) +
                        (ball_pos.y() - target_.y()) * (ball_pos.x() - robot.x());
        position = std::signbit(tc) == std::signbit(tp) ? tmp1 : tmp2;
      }
    }
  } else {
    //ボールがはやければ予測位置に行く

    //内積を使って導出
    //ボールの速度を正規化
    const auto normalize = ball_vec.normalized();
    //対象とreceiverの距離
    const auto length = robot - ball_pos;
    //内積より,対象と自分の直交する位置
    const auto dot = normalize.dot(length);
    //目標位置に変換
    position = (ball_pos + dot * normalize);

    //キックフラグは蹴らせない
    command.set_kick_flag(model::command::kick_flag_t{model::command::kick_type_t::none, 0});
    //捕球させる
    command.set_dribble(9);

    //自身のスピードが早いかボールより手前ならタンケッテのあれ
    if (std::signbit(ball_vec.dot(position - ball_pos))) {
      position = ball;
    }
  }

  //目標位置からベクトルへ
  const auto theta =
      util::wrap_to_pi(std::atan2(ball_pos.y() - robot.y(), ball_pos.x() - robot.x()));
  const auto omega = (theta - robot_theta);

  command.set_position({position.x(), position.y(), theta});

  flag_ = false;
  return command;
}

bool get_ball::finished() const {
  return flag_;
}

} // namespace ai_server
} // namespace game
} // namespace action
