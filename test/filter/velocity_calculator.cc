#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE velocity_calculator_test
#include <boost/test/unit_test.hpp>
#include "ai_server/filter/velocity_calculator.h"

BOOST_AUTO_TEST_SUITE(velocity_calculator)

//<model::ball>時のテスト
BOOST_AUTO_TEST_CASE(test01, *boost::unit_test::tolerance(0.0005)) {
  ai_server::filter::velocity_calculator<ai_server::model::ball> test_calculator;
  using namespace std::chrono_literals;
  //(0,0)から(5,10)へ1秒で移動
  ai_server::model::ball test_ball{5.0, 10.0, 0.0};
  const auto t1 = std::chrono::high_resolution_clock::time_point{};
  const auto t2 = t1 + 1s;
  test_calculator.apply(test_ball, t2);
  BOOST_TEST(test_ball.vx() == 5.0);
  BOOST_TEST(test_ball.vy() == 10.0);
  //(5,10)から(9,4)へ2秒で移動
  test_ball     = ai_server::model::ball{9.0, 4.0, 0.0};
  const auto t3 = t2 + 2s;
  test_calculator.apply(test_ball, t3);
  BOOST_TEST(test_ball.vx() == 2.0);
  BOOST_TEST(test_ball.vy() == -3.0);
}

//<model::robot>時のテスト
BOOST_AUTO_TEST_CASE(test02, *boost::unit_test::tolerance(0.0005)) {
  ai_server::filter::velocity_calculator<ai_server::model::robot> test_calculator;
  using namespace std::chrono_literals;
  //(0,0)から(3,6)へ1秒で移動、thetaが0.0から1.0へ1秒で変化
  ai_server::model::robot test_robot{0, 3.0, 6.0, 1.0};
  const auto t1 = std::chrono::high_resolution_clock::time_point{};
  const auto t2 = t1 + 1s;
  test_calculator.apply(test_robot, t2);
  BOOST_TEST(test_robot.vx() == 3.0);
  BOOST_TEST(test_robot.vy() == 6.0);
  BOOST_TEST(test_robot.omega() == 1.0);
  //(3,6)から(-3,9)へ3秒で移動、thetaが1.0から-3.0へ3秒で変化
  test_robot    = ai_server::model::robot{0, -3.0, 9.0, -3.0};
  const auto t3 = t2 + 3s;
  test_calculator.apply(test_robot, t3);
  BOOST_TEST(test_robot.vx() == -2.0);
  BOOST_TEST(test_robot.vy() == 1.0);
  BOOST_TEST(test_robot.omega() == 0.7610);
  //(-3,9)から(-3,9)へ1秒で移動、thetaが-3.0から3.0へ3秒で変化
  test_robot    = ai_server::model::robot{0, -3.0, 9.0, 3.0};
  const auto t4 = t3 + 1s;
  test_calculator.apply(test_robot, t4);
  BOOST_TEST(test_robot.vx() == 0.0);
  BOOST_TEST(test_robot.vy() == 0.0);
  BOOST_TEST(test_robot.omega() == -0.2831);
}

// reset()のテスト
BOOST_AUTO_TEST_CASE(test03, *boost::unit_test::tolerance(0.0005)) {
  ai_server::filter::velocity_calculator<ai_server::model::ball> test_calculator;
  using namespace std::chrono_literals;
  //(0,0)から(5,10)へ1秒で移動
  ai_server::model::ball test_ball{2000.0, -4500.0, 0.0};
  const auto t1 = std::chrono::high_resolution_clock::time_point{};
  const auto t2 = t1 + 1s;
  test_calculator.apply(test_ball, t2);
  BOOST_TEST(test_ball.vx() == 2000.0);
  BOOST_TEST(test_ball.vy() == -4500.0);
  // prev_state_を初期化
  test_calculator.reset();
  //(0,0)から(3,6)へ2秒で移動
  test_ball     = ai_server::model::ball{-6000.0, 3000.0, 0.0};
  const auto t3 = t2 + 2s;
  test_calculator.apply(test_ball, t3);
  BOOST_TEST(test_ball.vx() == -3000.0);
  BOOST_TEST(test_ball.vy() == 1500.0);
}

BOOST_AUTO_TEST_SUITE_END()