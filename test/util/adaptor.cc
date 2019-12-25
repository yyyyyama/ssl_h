#define BOOST_TEST_DYN_LINK

#include <algorithm>
#include <iterator>
#include <unordered_map>
#include <vector>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/util/adaptor.h"

using namespace ai_server;
namespace adaptors = boost::adaptors;

// テストで使う列挙子
enum class alphabet : int { a = 0, b = -1, c = -2, d = -3, e = -4, f = -5 };

BOOST_AUTO_TEST_SUITE(adaptor)

BOOST_AUTO_TEST_CASE(as_container_test) {
  { // vector

    // 元の値
    const std::vector<int> master{1, 2, 3, 5, 7, 11};

    const std::vector<int> result = master                                                   //
                                    | adaptors::transformed([](auto&& a) { return a * 11; }) //
                                    | util::as_container;

    // 正しい値
    const std::vector<int> correct_value{11, 22, 33, 55, 77, 121};

    BOOST_TEST(result == correct_value, boost::test_tools::per_element());
  }

  { // unordered_map

    // 元の値
    const std::unordered_map<int, alphabet> master{{1, alphabet::a}, {2, alphabet::b},
                                                   {3, alphabet::c}, {5, alphabet::d},
                                                   {7, alphabet::e}, {11, alphabet::f}};

    const std::unordered_map<alphabet, int> result =
        master                                                                              //
        | adaptors::transformed([](auto&& a) { return std::make_pair(a.second, a.first); }) //
        | util::as_container;

    BOOST_TEST(result.size() == master.size());
    for (auto&& [key, val] : master) {
      BOOST_TEST(result.at(val) == key);
    }
  }
}

BOOST_AUTO_TEST_CASE(to_container_test) {
  { // vector

    // 元の値
    const std::vector<int> master{1, 2, 3, 5, 7, 11};

    const std::vector<int> result = util::to_container( //
        adaptors::transform(master, [](auto&& a) { return a * 11; }));

    // 正しい値
    const std::vector<int> correct_value{11, 22, 33, 55, 77, 121};

    BOOST_TEST(result.size() == correct_value.size());

    BOOST_TEST(result == correct_value, boost::test_tools::per_element());
  }

  { // unordered_map

    // 元の値
    const std::unordered_map<int, alphabet> master{{1, alphabet::a}, {2, alphabet::b},
                                                   {3, alphabet::c}, {5, alphabet::d},
                                                   {7, alphabet::e}, {11, alphabet::f}};

    const std::unordered_map<alphabet, int> result = //
        util::to_container(                          //
            adaptors::transform(                     //
                master, [](auto&& a) { return std::make_pair(a.second, a.first); }));

    BOOST_TEST(result.size() == master.size());
    for (auto&& [key, val] : master) {
      BOOST_TEST(result.at(val) == key);
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
