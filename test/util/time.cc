#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai_server/util/time.h"

using namespace ai_server;
using namespace std::chrono_literals;

BOOST_AUTO_TEST_SUITE(time_utils)

BOOST_AUTO_TEST_CASE(to_duration) {
  // メタ関数のテスト
  static_assert(!util::detail::is_duration_v<int>, "");
  static_assert(util::detail::is_duration_v<std::chrono::duration<double>>, "");
  static_assert(util::detail::is_duration_v<std::chrono::microseconds>, "");

  // コンパイル時
  constexpr auto t1 = util::to_duration(5.004003);
  static_assert(t1 == std::chrono::duration_cast<std::chrono::microseconds>(5s + 4ms + 3us),
                "");
  static_assert(t1.count() == 5004003, "");

  // 実行時
  const double t2 = 10.009008;
  BOOST_TEST(util::to_duration(t2).count() == 10009008);
}

BOOST_AUTO_TEST_SUITE_END()
