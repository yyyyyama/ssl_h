#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/ball.h"

BOOST_AUTO_TEST_SUITE(ball_datatype)

BOOST_AUTO_TEST_CASE(test01) {
  // constructor
  ai_server::model::ball b{1.0, 2.0, 3.0};

  // get value
  BOOST_TEST(b.x() == 1.0);
  BOOST_TEST(b.y() == 2.0);
  BOOST_TEST(b.z() == 3.0);
}

BOOST_AUTO_TEST_CASE(test02) {
  ai_server::model::ball b{};

  // init
  BOOST_TEST(b.x() == 0.0);
  BOOST_TEST(b.y() == 0.0);
  BOOST_TEST(b.z() == 0.0);
  BOOST_TEST(b.is_lost() == true);

  // set x
  b.set_x(4.0);
  BOOST_TEST(b.x() == 4.0);
  // set y
  b.set_y(5.0);
  BOOST_TEST(b.y() == 5.0);
  // set z
  b.set_z(6.0);
  BOOST_TEST(b.z() == 6.0);
  // set vx
  b.set_vx(7.0);
  BOOST_TEST(b.vx() == 7.0);
  // set vy
  b.set_vy(8.0);
  BOOST_TEST(b.vy() == 8.0);
  // set ax
  b.set_ax(9.0);
  BOOST_TEST(b.ax() == 9.0);
  // set ay
  b.set_ay(10.0);
  BOOST_TEST(b.ay() == 10.0);
  // set is_lost
  b.set_is_lost(false);
  BOOST_TEST(b.is_lost() == false);
}

BOOST_AUTO_TEST_CASE(estimator) {
  using namespace std::chrono_literals;

  ai_server::model::ball b1{};

  // model::ball 初期化後は推定関数は設定されていない
  BOOST_TEST(!b1.has_estimator());
  // 推定関数が設定されていないときの推定結果は nullopt
  BOOST_TEST(!b1.state_after(0s).has_value());

  b1.set_x(1);
  b1.set_y(2);
  b1.set_z(3);

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
  BOOST_TEST(r1->z() == 3.0);

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
