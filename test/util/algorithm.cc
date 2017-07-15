#define BOOST_TEST_DYN_LINK

#include <queue>
#include <stack>
#include <vector>
#include <boost/test/unit_test.hpp>

#include "ai_server/util/algorithm.h"

using namespace ai_server;

BOOST_AUTO_TEST_SUITE(algorithm)

BOOST_AUTO_TEST_CASE(pop_each) {
  const std::vector<int> v1{1, 2, 3, 4, 5, 6, 7};
  const std::vector<int> v2{7, 6, 5, 4, 3, 2, 1};

  std::priority_queue<int> pq;
  std::queue<int> q;
  std::stack<int> s;

  // 各コンテナに1~7の値を push
  for (int n : v1) {
    pq.push(n);
    q.push(n);
    s.push(n);
  }

  std::vector<int> res;

  util::pop_each(pq, [&res](auto&& n) { res.emplace_back(n); });
  // res と v2 が一致するか
  BOOST_TEST(res == v2, boost::test_tools::per_element());
  // pq は空か
  BOOST_TEST(pq.empty());

  res.clear();

  util::pop_each(q, [&res](auto&& n) { res.emplace_back(n); });
  // res と v1 が一致するか
  BOOST_TEST(res == v1, boost::test_tools::per_element());
  // q は空か
  BOOST_TEST(q.empty());

  res.clear();

  util::pop_each(s, [&res](auto&& n) { res.emplace_back(n); });
  // res と v2 が一致するか
  BOOST_TEST(res == v2, boost::test_tools::per_element());
  // s は空か
  BOOST_TEST(s.empty());

  res.clear();

  // 空の queue
  std::queue<int> eq;
  util::pop_each(eq, [&res](auto&& n) { res.emplace_back(n); });
  // res は空か
  BOOST_TEST(res.empty());

  // 空の stack
  std::stack<int> es;
  util::pop_each(es, [&res](auto&& n) { res.emplace_back(n); });
  // res は空か
  BOOST_TEST(res.empty());
}

BOOST_AUTO_TEST_SUITE_END()
