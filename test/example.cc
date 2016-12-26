#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE ExampleTest

#include <boost/test/unit_test.hpp>

constexpr int factorial(int n) {
  return n <= 1 ? 1 : (n * factorial(n - 1));
}

BOOST_AUTO_TEST_SUITE(example)

BOOST_AUTO_TEST_CASE(test001) {
  constexpr int fact = factorial(10);
  BOOST_TEST(fact == 3628800);
}

BOOST_AUTO_TEST_SUITE_END()
