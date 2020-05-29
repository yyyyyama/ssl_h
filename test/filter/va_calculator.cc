#define BOOST_TEST_DYN_LINK

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/robot.h"
#include "ai_server/filter/va_calculator.h"

using namespace std::chrono_literals;

namespace filter = ai_server::filter;
namespace model  = ai_server::model;

BOOST_AUTO_TEST_SUITE(va_calculator)

BOOST_AUTO_TEST_CASE(robot, *boost::unit_test::tolerance(0.0000001)) {
  namespace bmc = boost::math::double_constants;

  // 初期時刻
  constexpr auto t = std::chrono::system_clock::time_point{};

  // 初期位置 (100, 200, pi/6)
  model::robot r{};
  r.set_x(100);
  r.set_y(200);
  r.set_theta(bmc::two_thirds_pi);
  r.set_vx(0);
  r.set_vy(0);
  r.set_omega(0);
  r.set_ax(0);
  r.set_ay(0);
  r.set_alpha(0);

  filter::va_calculator<model::robot> f;

  {
    const auto rf = f.update(r, t);

    // 最初は速度, 加速度は計算されない
    BOOST_TEST(rf->x() == 100);
    BOOST_TEST(rf->y() == 200);
    BOOST_TEST(rf->theta() == bmc::two_thirds_pi);
    BOOST_TEST(rf->vx() == 0);
    BOOST_TEST(rf->vy() == 0);
    BOOST_TEST(rf->omega() == 0);
    BOOST_TEST(rf->ax() == 0);
    BOOST_TEST(rf->ay() == 0);
    BOOST_TEST(rf->alpha() == 0);
  }

  r.set_x(200);
  r.set_y(0);
  r.set_theta(bmc::third_pi);

  {
    const auto rf = f.update(r, t + 1s);

    BOOST_TEST(rf->x() == 200);
    BOOST_TEST(rf->y() == 0);
    BOOST_TEST(rf->theta() == bmc::third_pi);
    BOOST_TEST(rf->vx() == 100);
    BOOST_TEST(rf->vy() == -200);
    BOOST_TEST(rf->omega() == -bmc::third_pi);
  }

  r.set_x(600);
  r.set_y(-300);
  r.set_theta(bmc::pi + bmc::two_thirds_pi); // 境界を超えてみる

  {
    const auto rf = f.update(r, t + 2s);

    BOOST_TEST(rf->x() == 600);
    BOOST_TEST(rf->y() == -300);
    BOOST_TEST(rf->theta() == (bmc::pi + bmc::two_thirds_pi));
    BOOST_TEST(rf->vx() == 400);
    BOOST_TEST(rf->vy() == -300);
    BOOST_TEST(rf->omega() == -bmc::two_thirds_pi);
    BOOST_TEST(rf->ax() == 300);
    BOOST_TEST(rf->ay() == -100);
    BOOST_TEST(rf->alpha() == -bmc::third_pi);
  }

  {
    // timeを変化させずに更新してみる
    const auto rf = f.update(r, t + 2s);

    // 直前と同じ値が返る
    BOOST_TEST(rf->x() == 600);
    BOOST_TEST(rf->y() == -300);
    BOOST_TEST(rf->theta() == (bmc::pi + bmc::two_thirds_pi));
    BOOST_TEST(rf->vx() == 400);
    BOOST_TEST(rf->vy() == -300);
    BOOST_TEST(rf->omega() == -bmc::two_thirds_pi);
    BOOST_TEST(rf->ax() == 300);
    BOOST_TEST(rf->ay() == -100);
    BOOST_TEST(rf->alpha() == -bmc::third_pi);
  }

  {
    // nullopt を渡したときの結果はnullopt
    const auto rf = f.update(std::nullopt, t + 3s);
    BOOST_TEST(!rf.has_value());
  }

  {
    const auto rf = f.update(r, t + 4s);

    // 一度ロストした後は速度, 加速度は計算されない
    BOOST_TEST(rf->x() == 600);
    BOOST_TEST(rf->y() == -300);
    BOOST_TEST(rf->theta() == (bmc::pi + bmc::two_thirds_pi));
    BOOST_TEST(rf->vx() == 0);
    BOOST_TEST(rf->vy() == 0);
    BOOST_TEST(rf->omega() == 0);
    BOOST_TEST(rf->ax() == 0);
    BOOST_TEST(rf->ay() == 0);
    BOOST_TEST(rf->alpha() == 0);
  }
}

BOOST_AUTO_TEST_SUITE_END()
