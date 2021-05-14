#define BOOST_TEST_DYN_LINK

#include <stdexcept>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/world.h"

#include "ssl-protos/vision_detection.pb.h"

BOOST_AUTO_TEST_SUITE(world_model)

BOOST_AUTO_TEST_CASE(nyan) {
  ai_server::model::world w;

  {
    // 保持するメンバが正しく初期化されているか

    ai_server::model::field f{};
    BOOST_TEST(w.field().length() == f.length());
    BOOST_TEST(w.field().width() == f.width());
    BOOST_TEST(w.field().center_radius() == f.center_radius());
    BOOST_TEST(w.field().goal_width() == f.goal_width());
    BOOST_TEST(w.field().penalty_length() == f.penalty_length());
    BOOST_TEST(w.field().penalty_width() == f.penalty_width());

    ai_server::model::ball b{};
    BOOST_TEST(w.ball().x() == b.x());
    BOOST_TEST(w.ball().y() == b.y());
    BOOST_TEST(w.ball().z() == b.z());

    BOOST_TEST(w.robots_blue().size() == 0);
    BOOST_TEST(w.robots_yellow().size() == 0);
  }

  {
    // getter/setterが正しく動作するか

    ai_server::model::field field{};
    field.set_length(1);
    w.set_field(field);

    ai_server::model::ball ball{123, 456, 789};
    w.set_ball(ball);

    ai_server::model::world::robots_list robots_blue{{1, {}}, {3, {}}};
    w.set_robots_blue(robots_blue);

    ai_server::model::world::robots_list robots_yellow{{2, {}}, {4, {}}};
    w.set_robots_yellow(robots_yellow);

    BOOST_TEST(w.field().length() == 1);
    BOOST_TEST(w.ball().x() = 123);
    BOOST_TEST(w.robots_blue().size() == 2);
    BOOST_TEST(w.robots_blue().count(1) == 1);
    BOOST_TEST(w.robots_blue().count(3) == 1);
    BOOST_TEST(w.robots_yellow().size() == 2);
    BOOST_TEST(w.robots_yellow().count(2) == 1);
    BOOST_TEST(w.robots_yellow().count(4) == 1);

    ai_server::model::world w2{std::move(field), std::move(ball), std::move(robots_blue),
                               std::move(robots_yellow)};
    BOOST_TEST(w2.field().length() == 1);
    BOOST_TEST(w2.ball().x() = 123);
    BOOST_TEST(w2.robots_blue().size() == 2);
    BOOST_TEST(w2.robots_blue().count(1) == 1);
    BOOST_TEST(w2.robots_blue().count(3) == 1);
    BOOST_TEST(w2.robots_yellow().size() == 2);
    BOOST_TEST(w2.robots_yellow().count(2) == 1);
    BOOST_TEST(w2.robots_yellow().count(4) == 1);
  }

  {
    auto check = [](const auto& copy, const auto& original) {
      BOOST_TEST(copy.field().length() == original.field().length());
      BOOST_TEST(copy.ball().x() = original.ball().x());
      BOOST_TEST(copy.robots_blue().size() == original.robots_blue().size());
      BOOST_TEST(copy.robots_blue().count(1) == original.robots_blue().count(1));
      BOOST_TEST(copy.robots_blue().count(3) == original.robots_blue().count(3));
      BOOST_TEST(copy.robots_yellow().size() == original.robots_yellow().size());
      BOOST_TEST(copy.robots_yellow().count(1) == original.robots_yellow().count(1));
      BOOST_TEST(copy.robots_yellow().count(3) == original.robots_yellow().count(3));
    };

    // コピーコンストラクタが正しく動作するか
    ai_server::model::world w2(w);
    check(w2, w);

    // コピー代入演算子が正しく動作するか
    ai_server::model::world w3;
    w3 = w;
    check(w3, w);

    // コピーなのでwを変更してもw2, w3は変化しない
    const auto prev_ball = w.ball();
    w.set_ball({12, 34, 56});
    BOOST_TEST(w2.ball().x() = 123);
    BOOST_TEST(w3.ball().x() = 123);
    w.set_ball(prev_ball);

    // ムーブコンストラクタが正しく動作するか
    ai_server::model::world w4(std::move(w3));
    check(w4, w);

    // ムーブ代入演算子が正しく動作するか
    ai_server::model::world w5;
    w5 = std::move(w4);
    check(w5, w);
  }
}

BOOST_AUTO_TEST_CASE(helper_functions) {
  ai_server::model::world w{};
  // blue には ID1 が1台
  w.set_robots_blue({{1, {}}});
  // yellow には ID2,ID4 が1台ずつ
  w.set_robots_yellow({{2, {}}, {4, {}}});

  const auto rb = w.robots_blue();
  const auto ry = w.robots_yellow();

  {
    const auto r = ai_server::model::our_robots(w, ai_server::model::team_color::blue);
    BOOST_TEST(r.size() == rb.size());
    for (auto&& [k, v] : rb) {
      BOOST_TEST(r.count(k) == 1);
    }
  }

  {
    const auto r = ai_server::model::our_robots(w, ai_server::model::team_color::yellow);
    BOOST_TEST(r.size() == ry.size());
    for (auto&& [k, v] : ry) {
      BOOST_TEST(r.count(k) == 1);
    }
  }

  {
    const auto r = ai_server::model::enemy_robots(w, ai_server::model::team_color::blue);
    BOOST_TEST(r.size() == ry.size());
    for (auto&& [k, v] : ry) {
      BOOST_TEST(r.count(k) == 1);
    }
  }

  {
    const auto r = ai_server::model::enemy_robots(w, ai_server::model::team_color::yellow);
    BOOST_TEST(r.size() == rb.size());
    for (auto&& [k, v] : rb) {
      BOOST_TEST(r.count(k) == 1);
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
