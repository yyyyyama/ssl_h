#define BOOST_TEST_DYN_LINK

#include <cmath>
#include <memory>
#include <unordered_map>

#include <boost/test/unit_test.hpp>

#include "ai_server/game/action/with_planner.h"
#include "ai_server/game/context.h"
#include "ai_server/game/nnabla.h"
#include "ai_server/planner/base.h"
#include "ai_server/planner/obstacle_list.h"

namespace action  = ai_server::game::action;
namespace game    = ai_server::game;
namespace model   = ai_server::model;
namespace planner = ai_server::planner;

struct mock_planner : public planner::base {
  Eigen::Vector2d from;
  Eigen::Vector2d to;

  virtual planner::base::planner_type planner() override {
    return [this](const Eigen::Vector2d& f, const Eigen::Vector2d& t,
                  const planner::obstacle_list&) {
      from = f;
      to   = t;
      return planner::base::result_type{f + Eigen::Vector2d(10, 20), 1.23};
    };
  }
};

struct stub_action : public action::base {
  stub_action(game::context& ctx, unsigned int id) : base{ctx, id}, cmd{} {}

  bool f;
  bool finished() const override {
    return f;
  }

  model::command cmd;
  model::command execute() override {
    return cmd;
  }
};

BOOST_AUTO_TEST_SUITE(with_planner)

BOOST_AUTO_TEST_CASE(execute) {
  game::context ctx{};
  {
    ctx.team_color = model::team_color::yellow;
    ctx.world.set_robots_yellow({
        {123, {100, 200, 300}},
    });
  }

  auto a  = std::make_shared<stub_action>(ctx, 123);
  auto pp = std::make_unique<mock_planner>();
  auto& p = *pp;
  auto b  = std::make_shared<action::with_planner>(a, std::move(pp), planner::obstacle_list{});

  {
    BOOST_TEST(a->id() == 123);
    BOOST_TEST(b->id() == 123);

    a->f = true;
    BOOST_TEST(a->finished());
    BOOST_TEST(b->finished());

    a->f = false;
    BOOST_TEST(!a->finished());
    BOOST_TEST(!b->finished());
  }

  // setpoint が速度のときは setpoint の大きさで planner が算出した位置の方向の速度が出力される
  {
    a->cmd.set_velocity(3, 4);
    a->cmd.set_angle(5);
    const auto cmd = b->execute();

    // planner の引数に現在地が渡されている
    BOOST_TEST(p.from.x() == 100);
    BOOST_TEST(p.from.y() == 200);

    // planner を通した値が出力される
    // setpoint は速度・角度が混在していても Ok
    auto sp = cmd.setpoint_pair();
    auto& v = std::get<model::setpoint::velocity>(std::get<0>(sp));
    auto& a = std::get<model::setpoint::angle>(std::get<1>(sp));
    BOOST_TEST(std::get<0>(v) == std::hypot(3, 4) * 10 / std::hypot(10, 20));
    BOOST_TEST(std::get<1>(v) == std::hypot(3, 4) * 20 / std::hypot(10, 20));
    BOOST_TEST(std::get<0>(a) == 5);
  }

  // setpoint が座標のときは planner が算出した位置が出力される
  {
    a->cmd.set_position(4, 5);
    a->cmd.set_velanglar(6);
    const auto cmd = b->execute();

    // planner の引数に現在地、目標座標が渡されている
    BOOST_TEST(p.from.x() == 100);
    BOOST_TEST(p.from.y() == 200);
    BOOST_TEST(p.to.x() == 4);
    BOOST_TEST(p.to.y() == 5);

    // planner を通した値が出力される
    // setpoint は座標・角速度が混在していても Ok
    auto sp  = cmd.setpoint_pair();
    auto& p  = std::get<model::setpoint::position>(std::get<0>(sp));
    auto& va = std::get<model::setpoint::velangular>(std::get<1>(sp));
    BOOST_TEST(std::get<0>(p) == 100 + 10);
    BOOST_TEST(std::get<1>(p) == 200 + 20);
    BOOST_TEST(std::get<0>(va) == 6);
  }
}

BOOST_AUTO_TEST_SUITE_END()
