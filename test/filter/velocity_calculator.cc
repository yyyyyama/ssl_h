#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE velocity_calculator_test
#include <boost/test/unit_test.hpp>
#include "ai_server/filter/velocity_calculator.h"

#include <thread>

BOOST_AUTO_TEST_SUITE(velocity_calculator)

//<model::ball>時のテスト
BOOST_AUTO_TEST_CASE(test01) {
  ai_server::filter::velocity_calculator<ai_server::model::ball> test_calculator;
 
  //(0,0)から(5,10)へ1秒で移動
  ai_server::model::ball test_ball{5.0, 10.0, 0.0};  
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::chrono::high_resolution_clock::time_point test_time = std::chrono::high_resolution_clock::now();
  test_calculator.apply(test_ball, test_time);
  BOOST_TEST(test_ball.vx() < 5.05);
  BOOST_TEST(test_ball.vx() > 4.95);
  BOOST_TEST(test_ball.vy() < 10.05);
  BOOST_TEST(test_ball.vy() > 9.95);

  //(5,10)から(9,4)へ2秒で移動
  test_ball = ai_server::model::ball{9.0, 4.0, 0.0};  
  std::this_thread::sleep_for(std::chrono::seconds(2));
  test_time = std::chrono::high_resolution_clock::now();
  test_calculator.apply(test_ball, test_time);
  BOOST_TEST(test_ball.vx() < 2.05);
  BOOST_TEST(test_ball.vx() > 1.95);
  BOOST_TEST(test_ball.vy() < -2.05);
  BOOST_TEST(test_ball.vy() > -3.95);

}

//<model::robot>時のテスト
BOOST_AUTO_TEST_CASE(test02) {
  ai_server::filter::velocity_calculator<ai_server::model::robot> test_calculator;

  //(0,0)から(3,6)へ1秒で移動、thetaが0.0から1.0へ1秒で変化
  ai_server::model::robot test_robot{0, 3.0, 6.0, 1.0};  
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::chrono::high_resolution_clock::time_point test_time = std::chrono::high_resolution_clock::now();
  test_calculator.apply(test_robot, test_time);
  BOOST_TEST(test_robot.vx() < 3.05);
  BOOST_TEST(test_robot.vx() > 2.95);
  BOOST_TEST(test_robot.vy() < 6.05);
  BOOST_TEST(test_robot.vy() > 5.95);
  BOOST_TEST(test_robot.omega() < 1.05);
  BOOST_TEST(test_robot.omega() > 0.95);

  //(3,6)から(-3,9)へ3秒で移動、thetaが1.0から-2.0へ3秒で変化
  test_robot = ai_server::model::robot{0, -3.0, 9.0, -2.0};  
  std::this_thread::sleep_for(std::chrono::seconds(3));
  test_time = std::chrono::high_resolution_clock::now();
  test_calculator.apply(test_robot, test_time);
  BOOST_TEST(test_robot.vx() < -1.05);
  BOOST_TEST(test_robot.vx() > -2.95);
  BOOST_TEST(test_robot.vy() < 1.05);
  BOOST_TEST(test_robot.vy() > 0.95);
  BOOST_TEST(test_robot.omega() < -0.05);
  BOOST_TEST(test_robot.omega() > -1.95);
  
}

//reset()のテスト
BOOST_AUTO_TEST_CASE(test03){
  ai_server::filter::velocity_calculator<ai_server::model::ball> test_calculator;
  //(0,0)から(5,10)へ1秒で移動
  ai_server::model::ball test_ball{5.0, 10.0, 0.0};  
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::chrono::high_resolution_clock::time_point test_time = std::chrono::high_resolution_clock::now();
  test_calculator.apply(test_ball, test_time);
  BOOST_TEST(test_ball.vx() < 5.05);
  BOOST_TEST(test_ball.vx() > 4.95);
  BOOST_TEST(test_ball.vy() < 10.05);
  BOOST_TEST(test_ball.vy() > 9.95);

  //prev_ball_を初期化
  test_calculator.reset();
  //(0,0)から(3,6)へ1秒で移動
  test_ball = ai_server::model::ball{2.0, 4.0, 0.0};  
  std::this_thread::sleep_for(std::chrono::seconds(2));
  test_time = std::chrono::high_resolution_clock::now();
  test_calculator.apply(test_ball, test_time);
  BOOST_TEST(test_ball.vx() < 1.05);
  BOOST_TEST(test_ball.vx() > 0.95);
  BOOST_TEST(test_ball.vy() < 2.05);
  BOOST_TEST(test_ball.vy() > 1.95);

}
BOOST_AUTO_TEST_SUITE_END()