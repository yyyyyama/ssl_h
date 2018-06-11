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
    : get_ball(world, is_yellow, id, Eigen::Vector2d::Zero()) {}

get_ball::get_ball(const model::world& world, bool is_yellow, unsigned int id,
                   Eigen::Vector2d target)
    : base(world, is_yellow, id), target_(target), pow_(128), flag_(false), chip_(false) {}

void get_ball::set_target(double x, double y) {
  target_ = Eigen::Vector2d{x, y};
}

void get_ball::set_pow(double pow) {
  pow_ = pow;
}

void get_ball::set_chip(bool chip) {
  chip_ = chip;
}

Eigen::Vector2d get_ball::target() const {
  return target_;
}

double get_ball::pow() const {
  return pow_;
}

bool get_ball::chip() const {
  return chip_;
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
  const auto robot_theta = util::math::wrap_to_pi(my_robot.theta());

  const auto ball_pos = util::math::position(world_.ball());
  const auto ball_vec = util::math::velocity(world_.ball());
  // 3.0秒後のボールの位置
  const Eigen::Vector2d ball = ball_pos + ball_vec * 3.0;

  // don't get the ball which is (going) out of the field
  auto field        = world_.field();
  double ball_angle = std::atan2(ball_vec.y(), ball_vec.x());
  double ball_speed = std::hypot(ball_vec.y(), ball_vec.x());
  if ((ball_pos.x() > field.x_max() - 1000.0 && ball_speed > 1000.0 &&
       ball_angle > pi<double>() / 4 && ball_angle < 3 * pi<double>() / 4) ||
      (ball_pos.x() < field.x_min() + 1000.0 && ball_speed > 1000.0 &&
       ball_angle > -3 * pi<double>() / 4 && ball_angle < -pi<double>() / 4) ||
      (ball_pos.y() > field.y_max() - 1000.0 && ball_speed > 1000.0 &&
       ball_angle > -pi<double>() / 4 && ball_angle < pi<double>() / 4) ||
      (ball_pos.y() < field.y_max() + 1000.0 && ball_speed > 1000.0 &&
       ball_angle > 3 * pi<double>() / 4 && ball_angle < -3 * pi<double>() / 4) ||
      ball_pos.x() > field.x_max() + 100.0 || ball_pos.x() < field.x_min() - 100.0 ||
      ball_pos.y() > field.y_max() + 100.0 || ball_pos.y() < field.y_min() - 100.0) {
    command.set_velocity({0.0, 0.0, 0.0});
  } else {
    //移動目標
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
    bool is_position = true;

    auto theta = 0.0;
    if (ball_vec.norm() < 800.0) {
      //ロボットの中心から口までの距離
      auto radius = 140.0;
      //目標に向かって蹴る
      {
        //目標位置に向いたら蹴る
        const auto theta = std::atan2(target_.y() - robot.y(), target_.x() - robot.x());
        if (std::abs(util::math::wrap_to_pi(theta - robot_theta)) < pi<double>() / 31.0) {
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
            if (flag || chip_) {
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
          if ((ball_pos - robot).norm() < 2000) {
            command.set_dribble(9);
          } else {
            command.set_dribble(0);
          }
        }
      }
      //移動目標はボールのちょい後ろ
      {
        const auto ratio =
            radius / ((target_ - ball_pos).norm() + radius * 5); //ボール - 目標位置の比
        position = (-ratio * target_ + 1 * ball_pos) / (1 - ratio);
      }
      {
        //目標位置と自分の位置で四角を作る
        const auto mergin = 140.0;
        polygon poly;
        const auto tmp = util::math::calc_isosceles_vertexes(robot, position, mergin);

        bg::exterior_ring(poly) = boost::assign::list_of<point>(robot.x(), robot.y())(
            std::get<0>(tmp).x(), std::get<0>(tmp).y())(
            std::get<1>(tmp).x(), std::get<1>(tmp).y())(robot.x(), robot.y());

        const point p(ball_pos.x(), ball_pos.y());
        // 角度がいい感じか
        bool flag =
            (ball - robot).norm() < 300 &&
            std::abs(util::math::wrap_to_pi(vectorangle(ball - robot) - robot_theta)) > 0.5;
        //間にボールがあったら回り込む
        if (!bg::disjoint(p, poly) || flag) {
          //判定の幅
          const auto mergin = 500.0;

          const auto [tmp1, tmp2] =
              util::math::calc_isosceles_vertexes(robot, ball_pos, mergin);
          const auto b2r = vectorangle(robot - ball);
          const auto b2p = vectorangle(position - ball);
          position       = util::math::wrap_to_pi(b2p - b2r) < 0 ? tmp1 : tmp2;
        }
      }
      theta =
          util::math::wrap_to_pi(std::atan2(target_.y() - robot.y(), target_.x() - robot.x()));
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
      }

      //キックフラグは蹴らせない
      command.set_kick_flag(model::command::kick_flag_t{model::command::kick_type_t::none, 0});
      //捕球させる
      if ((ball_pos - robot).norm() < 2000) {
        command.set_dribble(9);
      } else {
        command.set_dribble(0);
      }

      //ボールより手前ならタンケッテのあれ
      if (std::signbit(ball_vec.dot(position - ball_pos))) {
        position = ball;
        {
          //目標位置と自分の位置で四角を作る
          const auto mergin = 500.0;
          polygon poly;
          const auto tmp = util::math::calc_isosceles_vertexes(robot, position, mergin);
          bg::exterior_ring(poly) = boost::assign::list_of<point>(robot.x(), robot.y())(
              std::get<0>(tmp).x(), std::get<0>(tmp).y())(
              std::get<1>(tmp).x(), std::get<1>(tmp).y())(robot.x(), robot.y());
          const point p(ball_pos.x(), ball_pos.y());

          bool flag = false;
          if ((ball_pos - robot).norm() < 2000) flag = true;

          //間にボールがあったら回り込む
          if (!bg::disjoint(p, poly) && flag) {
            //判定の幅
            const auto mergin = 800.0;

            const auto [tmp1, tmp2] =
                util::math::calc_isosceles_vertexes(robot, ball_pos, mergin);
            const auto b2r = vectorangle(robot - ball);
            const auto b2p = vectorangle(position - ball);
            position       = util::math::wrap_to_pi(b2p - b2r) < 0 ? tmp1 : tmp2;

            is_position = false;
            velocity    = position - robot;
          } else {
            const auto nb2r = vectorangle(robot - ball_pos);
            const auto nb2p = vectorangle(position - ball);
            // 当たらないようにする
            if (std::abs(util::math::wrap_to_pi(nb2r - nb2p)) < 0.1) {
              const auto margin = 200.0;
              const auto [tmp1, tmp2] =
                  util::math::calc_isosceles_vertexes(robot, ball_pos, mergin);
              const auto b2r = vectorangle(robot - ball);
              const auto b2p = vectorangle(position - ball);
              position       = util::math::wrap_to_pi(b2p - b2r) < 0 ? tmp1 : tmp2;
            }

            is_position = false;
            velocity    = position - robot;
          }
        }
      } else {
        is_position = false;
        velocity    = position - robot;
      }
      theta = util::math::wrap_to_pi(
          std::atan2(ball_pos.y() - robot.y(), ball_pos.x() - robot.x()));
    }

    //目標位置が外側に行ったらいい感じにする
    //出たラインとの交点に移動
    if (std::abs(position.y()) > world_.field().y_max()) {
      {
        is_position  = true;
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
        is_position  = true;
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

    if (is_position) {
      command.set_position({position.x(), position.y(), theta});
    } else {
      velocity =
          velocity.norm() < 500 ? Eigen::Vector2d(velocity / 2) : Eigen::Vector2d(velocity);
      command.set_velocity({velocity.x(), velocity.y(), theta - robot_theta});
    }
  }
  flag_ = false;
  return command;
}

bool get_ball::finished() const {
  return flag_;
}
// ベクトルを渡すと角度を返してくれるもの{{{
double get_ball::vectorangle(Eigen::Vector2d vec) const {
  return std::atan2(vec.y(), vec.x());
} // }}}

} // namespace action
} // namespace game
} // namespace ai_server
