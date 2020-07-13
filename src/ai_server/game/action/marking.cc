#include <cmath>
#include <Eigen/Dense>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/assign/list_of.hpp>

#include "ai_server/game/action/marking.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/geometry.h"

namespace ai_server {
namespace game {
namespace action {

marking::marking(context& ctx, unsigned int id)
    : base(ctx, id), mode_(mark_mode::kick_block), flag_(false), radius_(250.0) {}

void marking::mark_robot(unsigned int enemy_id) {
  enemy_id_ = enemy_id;
}
void marking::set_mode(mark_mode mode) {
  mode_ = mode;
}
void marking::set_radius(double radius) {
  radius_ = radius;
}
model::command marking::execute() {
  namespace bg  = boost::geometry;
  using point   = bg::model::d2::point_xy<double>;
  using polygon = bg::model::polygon<point>;
  using boost::math::constants::pi;

  //それぞれ自機と敵機を生成
  model::command ally_robot{};
  const auto enemy_robots = model::enemy_robots(world(), team_color());
  const auto my_robots    = model::our_robots(world(), team_color());
  //指定されたロボットが見えなかったらその位置で停止
  if (!enemy_robots.count(enemy_id_) || !my_robots.count(id_)) {
    ally_robot.set_velocity({0.0, 0.0, 0.0});
    return ally_robot;
  }
  const auto& enemy_robot = enemy_robots.at(enemy_id_);
  const auto& my_robot    = my_robots.at(id_);
  //必要なパラメータ
  const Eigen::Vector2d enemy{enemy_robot.x(), enemy_robot.y()};
  const Eigen::Vector2d my{my_robot.x(), my_robot.y()};
  const Eigen::Vector2d ball{world().ball().x(), world().ball().y()};
  const Eigen::Vector2d goal{world().field().x_min(), 0.0};
  Eigen::Vector2d tmp_pos{0.0, 0.0};
  Eigen::Vector2d position{0.0, 0.0};
  auto ratio = 0.0; //敵位置とボールの比
  auto tmp   = 0.0; //どうでもいい一時的な数値に使う
  switch (mode_) {
    case mark_mode::kick_block:                   //ボールを蹴るのを阻止(外分点)
      tmp      = (enemy - ball).norm() / radius_; //敵位置 - 自位置の比
      ratio    = 1 - tmp;
      position = (-ratio * enemy + ball) / tmp;
      tmp_pos  = ball;
      break;
    case mark_mode::corner_block:
      ratio    = radius_ / ((enemy - ball).norm() + radius_); //ボール - 目標位置の比
      position = (-ratio * enemy + 1 * ball) / (1 - ratio);
      tmp_pos  = ball;
      break;
    case mark_mode::shoot_block:                 //シュートを阻止
      const auto length = (goal - enemy).norm(); //敵機とゴールの距離
      const double penalty_rad =
          std::sqrt(std::pow(world().field().penalty_width() / 2.0, 2.0) +
                    std::pow(world().field().penalty_length(), 2.0)) +
          300.0;
      tmp = std::signbit(penalty_rad - length / 2)
                ? 0
                : penalty_rad - length / 2; //敵機-ゴール中央の中間地点とゴールラインの差
      ratio    = (length / 2 + tmp) / length;
      position = (1 - ratio) * goal + ratio * enemy;
      tmp_pos  = enemy;
  }
  {
    //移動経路に他のものがあったらよける
    //判定の幅
    const auto mergin1 = 300.0;
    const auto mergin2 = 1000.0;
    polygon poly;
    bg::exterior_ring(poly) = [&my, &position, &mergin1] {
      Eigen::Vector2d p1, p2;
      std::tie(p1, p2) = util::math::calc_isosceles_vertexes(my, position, mergin1);
      return boost::assign::list_of<point> //軌道の三角
          (my.x(), my.y())(p1.x(), p1.y())(p2.x(), p2.y())(my.x(), my.y());
    }();
    const point p_e(enemy.x(), enemy.y());
    const point p_b(ball.x(), ball.y());
    if (!bg::disjoint(p_e, poly)) {
      const auto tmp_e  = util::math::calc_isosceles_vertexes(my, enemy, mergin2);
      const auto tmp_e1 = std::get<0>(tmp_e);
      const auto tmp_e2 = std::get<1>(tmp_e);
      position          = (std::abs(tmp_e1.y()) < std::abs(tmp_e2.y())) ? tmp_e1 : tmp_e2;
    } else if (!bg::disjoint(p_b, poly)) {
      const auto tmp_b  = util::math::calc_isosceles_vertexes(my, ball, mergin2);
      const auto tmp_b1 = std::get<0>(tmp_b);
      const auto tmp_b2 = std::get<1>(tmp_b);
      position          = (std::abs(tmp_b1.y()) < std::abs(tmp_b2.y())) ? tmp_b1 : tmp_b2;
    }
  }
  //向きをボールの方へ
  const auto theta =
      std::atan2(position.y() - tmp_pos.y(), position.x() - tmp_pos.x()) + pi<double>();

  //目標位置が外側に行ったらその場で停止
  if (std::abs(position.x()) > world().field().x_max() ||
      std::abs(position.y()) > world().field().y_max()) {
    ally_robot.set_velocity({0.0, 0.0, 0.0});
    return ally_robot;
  }

  //計算した値を自機にセット
  ally_robot.set_position({position.x(), position.y(), theta});

  return ally_robot;
}
bool marking::finished() const {
  return flag_;
}
} // namespace action
} // namespace game
} // namespace ai_server
