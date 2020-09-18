#define BOOST_TEST_DYN_LINK

#include <vector>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "ai_server/filter/state_observer/robot.h"
#include "ai_server/model/robot.h"

using namespace std::chrono_literals;

namespace filter = ai_server::filter;
namespace model  = ai_server::model;

BOOST_AUTO_TEST_SUITE(state_observer_robot)

BOOST_AUTO_TEST_CASE(rest, *boost::unit_test::tolerance(0.1)) {
  // テストするロボットの位置
  const auto robot_positions = std::vector<model::robot>{
      // 中心
      {0, 0, 0},
      // コーナー
      {4500, 3000, 0},
      {4500, -3000, 0},
      {-4500, -3000, 0},
      {-4500, 3000, 0},
  };

  // 初期時刻
  auto t = std::chrono::system_clock::now();
  model::robot r;

  // 結果書き込み用の式
  auto wr = [&r](std::optional<model::robot> robot_) { r = *robot_; };

  std::recursive_mutex mutex{};
  filter::state_observer::robot obs{mutex, wr, 1s};

  for (const auto& robot : robot_positions) {
    // 状態オブザーバを20ms * 500回更新する
    for (auto i = 0u; i < 500; ++i) {
      t += 20ms;
      obs.set_raw_value(robot, t);
      obs.observe(0, 0);
    }

    // それっぽい値が返ってくるか
    BOOST_TEST(r.x() == robot.x());
    BOOST_TEST(r.y() == robot.y());
    BOOST_TEST(r.theta() == 0);
  }
}

BOOST_AUTO_TEST_CASE(move, *boost::unit_test::tolerance(5.0)) {
  // テストするロボットの移動速度
  const auto robot_velocities =
      std::vector<Eigen::Vector3d>{{0, 0, 0}, {0, 100, 0}, {100, 0, 0}, {100, 100, 0}};

  // 初期時刻
  auto t = std::chrono::system_clock::now();
  model::robot r;
  model::robot r2;

  // 結果書き込み用の式
  auto wr = [&r2](std::optional<model::robot> robot_) { r2 = *robot_; };

  std::recursive_mutex mutex{};
  filter::state_observer::robot obs{mutex, wr, 1s};

  for (auto vel : robot_velocities) {
    r.set_vx(vel.x());
    r.set_vy(vel.y());
    // 状態オブザーバを20ms * 1000回更新する
    for (auto i = 0u; i < 1000; ++i) {
      t += 20ms;
      obs.set_raw_value(r, t);
      obs.observe(vel.x(), vel.y());
      r.set_x(r.x() + vel.x() * 0.02);
      r.set_y(r.y() + vel.y() * 0.02);
    }

    // それっぽい値が返ってくるか
    BOOST_TEST(r2.x() == r.x());
    BOOST_TEST(r2.y() == r.y());
    BOOST_TEST(r2.vx() == r.vx());
    BOOST_TEST(r2.vy() == r.vy());
  }
}

BOOST_AUTO_TEST_SUITE_END()
