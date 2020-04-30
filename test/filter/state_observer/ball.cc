#define BOOST_TEST_DYN_LINK

#include <vector>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/ball.h"
#include "ai_server/filter/state_observer/ball.h"

using namespace std::chrono_literals;

namespace filter = ai_server::filter;
namespace model  = ai_server::model;

BOOST_AUTO_TEST_SUITE(state_observer_ball)

BOOST_AUTO_TEST_CASE(rest, *boost::unit_test::tolerance(0.1)) {
  // テストするボールの位置
  const auto ball_positions = std::vector<model::ball>{
      // コーナー
      {4500, 3000, 0},
      {4500, -3000, 0},
      {-4500, -3000, 0},
      {-4500, 3000, 0},
      // 中心
      {0, 0, 0},
  };

  // 初期時刻
  auto t = std::chrono::system_clock::time_point{};

  filter::state_observer::ball obs{{}, t};

  for (auto&& ball : ball_positions) {
    model::ball b{};

    // 状態オブザーバを20ms * 500回更新する
    for (auto i = 0u; i < 500; ++i) {
      t += 20ms;
      b = obs.update(ball, t).value();
    }

    // それっぽい値が返ってくるか
    BOOST_TEST(b.x() == ball.x());
    BOOST_TEST(b.y() == ball.y());
  }
}

BOOST_AUTO_TEST_CASE(null) {
  auto t = std::chrono::system_clock::time_point{};
  filter::state_observer::ball obs{{}, t};

  // nullopt で更新したときの結果はnullopt
  const auto b = obs.update(std::nullopt, t);
  BOOST_TEST(!b.has_value());
}

BOOST_AUTO_TEST_SUITE_END()
