#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include "ai_server/model/robot.h"

BOOST_AUTO_TEST_SUITE(robotdata)

BOOST_AUTO_TEST_CASE(test001) {
  // constructor test
  ai_server::model::robot rob(2.0, 3.0, 4.0);

  // get value test
  BOOST_TEST(rob.x() == 2.0);
  BOOST_TEST(rob.y() == 3.0);
  BOOST_TEST(rob.theta() == 4.0);

  // set value test
  rob.set_x(10.0);
  BOOST_TEST(rob.x() == 10.0);

  rob.set_y(11.0);
  BOOST_TEST(rob.y() == 11.0);

  rob.set_vx(12.0);
  BOOST_TEST(rob.vx() == 12.0);

  rob.set_vy(13.0);
  BOOST_TEST(rob.vy() == 13.0);

  rob.set_theta(14.0);
  BOOST_TEST(rob.theta() == 14.0);

  rob.set_omega(15.0);
  BOOST_TEST(rob.omega() == 15.0);

  rob.set_ax(16.0);
  BOOST_TEST(rob.ax() == 16.0);

  rob.set_ay(17.0);
  BOOST_TEST(rob.ay() == 17.0);

  rob.set_alpha(18.0);
  BOOST_TEST(rob.alpha() == 18.0);
}

BOOST_AUTO_TEST_CASE(test002) {
  // constructor test
  ai_server::model::robot rob2{};

  // get value test
  BOOST_TEST(rob2.x() == 0.0);
  BOOST_TEST(rob2.y() == 0.0);
  BOOST_TEST(rob2.theta() == 0.0);

  // set value test
  rob2.set_x(40.0);
  BOOST_TEST(rob2.x() == 40.0);

  rob2.set_y(41.0);
  BOOST_TEST(rob2.y() == 41.0);

  rob2.set_vx(42.0);
  BOOST_TEST(rob2.vx() == 42.0);

  rob2.set_vy(43.0);
  BOOST_TEST(rob2.vy() == 43.0);

  rob2.set_theta(44.0);
  BOOST_TEST(rob2.theta() == 44.0);

  rob2.set_omega(45.0);
  BOOST_TEST(rob2.omega() == 45.0);
}

BOOST_AUTO_TEST_CASE(test003) {
  ai_server::model::robot r;

  BOOST_TEST(r.x() == 0.0);
  BOOST_TEST(r.y() == 0.0);
  BOOST_TEST(r.theta() == 0.0);
}

BOOST_AUTO_TEST_CASE(estimator) {
  using namespace std::chrono_literals;

  ai_server::model::robot b1{};

  // model::robot 初期化後は推定関数は設定されていない
  BOOST_TEST(!b1.has_estimator());
  // 推定関数が設定されていないときの推定結果は nullopt
  BOOST_TEST(!b1.state_after(0s).has_value());

  b1.set_x(1);
  b1.set_y(2);
  b1.set_theta(3);

  std::chrono::system_clock::duration f1t{};
  auto f1 = [&f1t](auto&& b, auto&& t) {
    f1t = t;
    return std::forward<decltype(b)>(b);
  };
  b1.set_estimator(f1);

  // 推定関数は設定したら true
  BOOST_TEST(b1.has_estimator());

  // 設定した推定関数が正しく呼ばれているか
  const auto r1 = b1.state_after(2s);
  BOOST_TEST(f1t.count() == std::chrono::system_clock::duration{2s}.count());
  BOOST_TEST(r1->x() == 1.0);
  BOOST_TEST(r1->y() == 2.0);
  BOOST_TEST(r1->theta() == 3.0);

  std::chrono::system_clock::duration f2t{};
  auto f2 = [&f2t](auto&&, auto&& t) {
    f2t = t;
    return std::nullopt;
  };
  b1.set_estimator(f2);

  // 推定関数は設定したら true
  BOOST_TEST(b1.has_estimator());

  const auto r2 = b1.state_after(4s);
  BOOST_TEST(f2t.count() == std::chrono::system_clock::duration{4s}.count());
  BOOST_TEST(!r2.has_value());

  // const なオブジェクトでも利用できるか
  const auto b2 = b1;
  const auto r3 = b2.state_after(6s);
  BOOST_TEST(f2t.count() == std::chrono::system_clock::duration{6s}.count());
  BOOST_TEST(!r3.has_value());

  // clear_estimator で推定関数が解除されるか
  b1.clear_estimator();
  BOOST_TEST(!b1.has_estimator());
}

BOOST_AUTO_TEST_SUITE_END()
