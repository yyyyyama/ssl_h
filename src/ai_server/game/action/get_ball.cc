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

void get_ball::set_pow(double pow) {
  pow_ = pow;
}

model::command get_ball::execute() {
  namespace bg  = boost::geometry;
  using point   = bg::model::d2::point_xy<double>;
  using polygon = bg::model::polygon<point>;
  using boost::math::constants::pi;

  model::command command(id_);

  const auto my_robots    = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  if (!my_robots.count(id_)) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  const auto& my_robot   = my_robots.at(id_);
  const auto robot       = util::math::position(my_robot);
  const auto robot_theta = util::wrap_to_pi(my_robot.theta());

  const auto ball_pos = util::math::position(world_.ball());
  const auto ball_vec = util::math::velocity(world_.ball());
  // 3.0秒後のボールの位置
  const Eigen::Vector2d ball = ball_pos + ball_vec * 3.0;

  //移動目標
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};

  auto theta = 0.0;
  if (ball_vec.norm() < 800.0) {
    //ロボットの中心から口までの距離
    auto radius = 140.0;
    //目標に向かって蹴る
    {
      //目標位置に向いたら蹴る
      const auto theta =
          util::wrap_to_pi(std::atan2(target_.y() - robot.y(), target_.x() - robot.x()));
      if (std::abs(theta - robot_theta) < pi<double>() / 31.0) {
        command.set_dribble(0);
        radius = 70;
        //間に敵がいるか判定
        {
          const auto radius = 1000;
          const auto tmp = (robot - target_).norm() / radius; //自位置 - 撃つ目標位置の比
          const auto ratio = 1 - tmp;
          //これで自分から1000の点が出せる
          decltype(robot) pos = (-ratio * robot + target_) / tmp;
          const auto mergin   = 200.0;
          const auto shift_p  = util::math::calc_isosceles_vertexes(pos, robot, mergin);
          polygon poly;
          bg::exterior_ring(poly) = boost::assign::list_of<point>(pos.x(), pos.y())(
              std::get<0>(shift_p).x(), std::get<0>(shift_p).y())(
              std::get<1>(shift_p).x(), std::get<1>(shift_p).y())(pos.x(), pos.y());
          bool flag = false;
          //自分から1000の位置と自分で三角形を作り,間に敵が1つでもあったらチップにする
          for (auto it : enemy_robots) {
            const point p((it.second).x(), (it.second).y());
            if (!bg::disjoint(p, poly)) {
              flag = true;
              break;
            }
          }
          if (flag) {
            command.set_kick_flag(
                model::command::kick_flag_t{model::command::kick_type_t::chip, pow_});
          } else {
            command.set_kick_flag(
                model::command::kick_flag_t{model::command::kick_type_t::line, pow_});
          }
        }
      } else {
        command.set_kick_flag(
            model::command::kick_flag_t{model::command::kick_type_t::none, 0});
        command.set_dribble(9);
      }
    }
    //移動目標はボールのちょい後ろ
    {
      const auto ratio =
          radius / ((target_ - ball_pos).norm() + radius); //ボール - 目標位置の比
      position = (-ratio * target_ + 1 * ball_pos) / (1 - ratio);
    }
    {
      //目標位置と自分の位置で四角を作る
      const auto mergin = 140.0;
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
            std::get<0>(util::math::calc_isosceles_vertexes(robot, ball_pos, mergin));
        const auto tmp2 =
            std::get<1>(util::math::calc_isosceles_vertexes(robot, ball_pos, mergin));

        const auto tc = (ball_pos.x() - target_.x()) * (tmp1.y() - ball_pos.y()) +
                        (ball_pos.y() - target_.y()) * (ball_pos.x() - tmp1.x());
        const auto tp = (ball_pos.x() - target_.x()) * (robot.y() - ball_pos.y()) +
                        (ball_pos.y() - target_.y()) * (ball_pos.x() - robot.x());
        position = std::signbit(tc) == std::signbit(tp) ? tmp1 : tmp2;
      }
    }
    theta = util::wrap_to_pi(std::atan2(target_.y() - robot.y(), target_.x() - robot.x()));
  } else {
    //ボールがはやければ予測位置に行く

    {
      //内積を使って導出
      //ボールの速度を正規化
      const auto normalize = ball_vec.normalized();
      //対象とreceiverの距離
      const auto length = robot - ball_pos;
      //内積より,対象と自分の直交する位置
      const auto dot = normalize.dot(length);
      //目標位置に変換
      position = (ball_pos + dot * normalize);
      //若干にゃーん
      // if ((robot - ball_pos).norm() < 1000.0) {
      //  const auto radius = 500.0;
      //  const auto ratio =
      //      radius / ((target_ - ball_pos).norm() + radius); //ボール - 目標位置の比
      //  position = (-ratio * target_ + 1 * ball_pos) / (1 - ratio);
      //}
    }

    //キックフラグは蹴らせない
    command.set_kick_flag(model::command::kick_flag_t{model::command::kick_type_t::none, 0});
    //捕球させる
    command.set_dribble(9);

    //ボールより手前ならタンケッテのあれ
    if (std::signbit(ball_vec.dot(position - ball_pos))) {
      position = ball;
      {
        //目標位置と自分の位置で四角を作る
        const auto mergin = 500.0;
        polygon poly;
        const auto tmp          = util::math::calc_isosceles_vertexes(robot, position, mergin);
        bg::exterior_ring(poly) = boost::assign::list_of<point>(robot.x(), robot.y())(
            std::get<0>(tmp).x(), std::get<0>(tmp).y())(
            std::get<1>(tmp).x(), std::get<1>(tmp).y())(robot.x(), robot.y());
        const point p(ball_pos.x(), ball_pos.y());
        //間にボールがあったら回り込む
        if (!bg::disjoint(p, poly)) {
          //判定の幅
          const auto mergin = 1000.0;
          const auto tmp1 =
              std::get<0>(util::math::calc_isosceles_vertexes(ball_pos, position, mergin));
          const auto tmp2 =
              std::get<1>(util::math::calc_isosceles_vertexes(ball_pos, position, mergin));

          const auto tc = (ball_pos.x() - position.x()) * (tmp1.y() - position.y()) +
                          (ball_pos.y() - position.y()) * (position.x() - tmp1.x());
          const auto tp = (ball_pos.x() - position.x()) * (robot.y() - position.y()) +
                          (ball_pos.y() - position.y()) * (position.x() - robot.x());
          position = std::signbit(tc) == std::signbit(tp) ? tmp1 : tmp2;
        }
      }
    }
    theta = util::wrap_to_pi(std::atan2(ball_pos.y() - robot.y(), ball_pos.x() - robot.x()));
  }

  //目標位置が外側に行ったらいい感じにする
  //出たラインとの交点に移動
  if (std::abs(position.y()) > world_.field().y_max()) {
    {
      const auto A = (position.y() - ball_pos.y()) / (position.x() - ball_pos.x());
      const auto B = ball_pos.y() - A * ball_pos.x();
      if (std::signbit(position.y()) == std::signbit(world_.field().y_min())) {
        position.y() = world_.field().y_min();
      } else {
        position.y() = world_.field().y_max();
      }
      position.x() = (position.y() - B) / A;
    }
  } else if (std::abs(position.x()) > world_.field().x_max()) {
    {
      const auto A = (position.y() - ball_pos.y()) / (position.x() - ball_pos.x());
      const auto B = ball_pos.y() - A * ball_pos.x();
      if (std::signbit(position.x()) == std::signbit(world_.field().x_min())) {
        position.x() = world_.field().x_min();
      } else {
        position.x() = world_.field().x_max();
      }
      position.y() = A * position.x() + B;
    }
  }

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
