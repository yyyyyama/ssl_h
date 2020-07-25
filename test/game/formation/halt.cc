#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai_server/game/action/no_operation.h"
#include "ai_server/game/context.h"
#include "ai_server/game/formation/halt.h"
#include "ai_server/game/nnabla.h"

namespace game = ai_server::game;

BOOST_AUTO_TEST_SUITE(halt)

BOOST_AUTO_TEST_CASE(basic) {
  const auto ids = std::vector<unsigned int>{1, 2, 3, 5, 7, 11};

  game::context ctx{};
  game::formation::halt h{ctx, ids};

  // 台数分の action が出力される
  auto actions = h.execute();
  BOOST_TEST(actions.size() == ids.size());
  for (std::size_t i = 0; i < ids.size(); ++i) {
    // action は no_operation
    auto a = std::dynamic_pointer_cast<game::action::no_operation>(actions.at(i));
    BOOST_TEST((a != nullptr));
    BOOST_TEST(a->id() == ids.at(i));
  }
}

BOOST_AUTO_TEST_SUITE_END()
