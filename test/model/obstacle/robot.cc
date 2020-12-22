#define BOOST_TEST_DYN_LINK

#include <boost/test/data/test_case.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/obstacle/robot.h"
#include "ai_server/model/world.h"

BOOST_AUTO_TEST_SUITE(robot)

namespace model    = ai_server::model;
namespace obstacle = model::obstacle;
using ai_server::planner::obstacle_list;

BOOST_DATA_TEST_CASE(add_robots_test, boost::unit_test::data::make({0.0, 500.0}), margin) {
  // 障害物リストに追加するロボット
  model::world::robots_list robots{
      {1, {}},
      {2, {524.23, 324.5, 1.433}},
      {3, {6.73, 651.43, 0.195}},
      {4, {321.03, 86.654, 5.433}},
      {5, {6565.546, 4355.564, 5.368}},
  };

  { // 任意の速度に設定
    double vx    = 0.0;
    double vy    = 0.0;
    double omega = 0.0;

    for (auto& r : robots) {
      r.second.set_vx(vx);
      r.second.set_vy(vy);
      r.second.set_omega(omega);
      vx += 74.0 - 3.6 * vx;
      vy += 2324.0 - 6.9 * vy;
      omega += 0.0534;
    }
  }

  // ロボットの情報のみを取り出す
  const auto ref_robots = [&robots]() {
    std::vector<model::robot> result;
    // 順序を保証するためにfor文を使う
    for (const auto& r : robots) {
      result.push_back(r.second);
    }
    return result;
  }();
  BOOST_TEST(ref_robots.size() == robots.size());

  { // add_robots(...)
    // 結果
    const auto o = [&robots, margin]() {
      obstacle_list list;
      obstacle::add_robots(list, robots, margin);
      return list.buffer();
    }();

    // サイズをチェック
    BOOST_TEST(o.size() == robots.size());

    // 障害物リストの各要素をチェック
    auto r_itr = ref_robots.begin();
    auto o_itr = o.begin();
    for (; r_itr != ref_robots.end() && o_itr != o.end(); ++r_itr, ++o_itr) {
      BOOST_TEST_CONTEXT("at " << std::distance(ref_robots.begin(), r_itr)) {
        const auto p = std::get_if<obstacle::point>(&o_itr->second);
        BOOST_TEST(p != nullptr);
        BOOST_TEST(p->geometry.x() == r_itr->x());
        BOOST_TEST(p->geometry.y() == r_itr->y());
        BOOST_TEST(p->margin == margin);
      }
    }
  }

  { // add_robot_paths(...)
    // 結果
    const auto o = [&robots, margin]() {
      using namespace std::chrono_literals;

      obstacle_list list;
      obstacle::add_robot_paths(list, robots, margin, -500ms, 2400ms);
      return list.buffer();
    }();

    // サイズをチェック
    BOOST_TEST(o.size() == robots.size());

    // 障害物リストの各要素をチェック
    auto r_itr = ref_robots.begin();
    auto o_itr = o.begin();
    for (; r_itr != ref_robots.end() && o_itr != o.end(); ++r_itr, ++o_itr) {
      BOOST_TEST_CONTEXT("at " << std::distance(ref_robots.begin(), r_itr)) {
        const auto p = std::get_if<obstacle::segment>(&o_itr->second);
        BOOST_TEST(p != nullptr);
        BOOST_TEST(std::get<0>(p->geometry).x() == r_itr->x() - 0.5 * r_itr->vx());
        BOOST_TEST(std::get<0>(p->geometry).y() == r_itr->y() - 0.5 * r_itr->vy());
        BOOST_TEST(std::get<1>(p->geometry).x() == r_itr->x() + 2.4 * r_itr->vx());
        BOOST_TEST(std::get<1>(p->geometry).y() == r_itr->y() + 2.4 * r_itr->vy());
        BOOST_TEST(p->margin == margin);
      }
    }
  }
}

BOOST_DATA_TEST_CASE(add_robots_if_test, boost::unit_test::data::make({0.0, 500.0}), margin) {
  // 障害物リストに追加するロボット
  const model::world::robots_list robots{
      {1, {}},
      {2, {524.23, 324.5, 1.433}},
      {3, {6.73, 651.43, 0.195}},
      {4, {321.03, 86.654, 5.433}},
      {5, {6565.546, 4355.564, 5.368}},
  };

  // 障害物リストに追加しないロボット
  const model::world::robots_list others{
      {11, {3546.8, -167.7, 0.546}},
      {12, {5436.23, 65.5, 1.564}},
      {13, {867.73, 768.43, 0.28}},
  };

  // 重複していないことを確認
  for (const auto& o : others) {
    BOOST_TEST_CONTEXT("at " << o.first) {
      BOOST_CHECK(std::none_of(robots.begin(), robots.end(),
                               [id = o.first](const auto& a) { return id == a.first; }));
    }
  }

  // 条件式
  auto pred = [&others](auto id, [[maybe_unused]] auto&& robot) {
    return others.count(id) == 0;
  };

  // 全てのロボット
  auto all = robots;
  all.insert(others.begin(), others.end());
  { // 任意の速度に設定
    double vx    = 0.0;
    double vy    = 0.0;
    double omega = 0.0;

    for (auto& r : all) {
      r.second.set_vx(vx);
      r.second.set_vy(vy);
      r.second.set_omega(omega);
      vx += 74.0 - 3.6 * vx;
      vy += 2324.0 + 6.9 * vy;
      omega += 0.0534;
    }
  }
  BOOST_TEST(all.size() == robots.size() + others.size());

  // ロボットの情報のみを取り出す
  const auto ref_robots = [&all, &pred]() {
    std::vector<model::robot> result;
    // 順序を保証するためにfor文を使う
    for (const auto& [id, robot] : all) {
      if (pred(id, robot)) result.push_back(robot);
    }
    return result;
  }();
  BOOST_TEST(ref_robots.size() == robots.size());

  { // add_robots_if(...)
    // 結果
    const auto o = [&all, &pred, margin]() {
      obstacle_list list;
      obstacle::add_robots_if(list, all, margin, pred);
      return list.buffer();
    }();

    // サイズをチェック
    BOOST_TEST(o.size() == robots.size());

    // 障害物リストの各要素をチェック
    auto r_itr = ref_robots.begin();
    auto o_itr = o.begin();
    for (; r_itr != ref_robots.end() && o_itr != o.end(); ++r_itr, ++o_itr) {
      BOOST_TEST_CONTEXT("at " << std::distance(ref_robots.begin(), r_itr)) {
        const auto p = std::get_if<obstacle::point>(&o_itr->second);
        BOOST_TEST(p != nullptr);
        BOOST_TEST(p->geometry.x() == r_itr->x());
        BOOST_TEST(p->geometry.y() == r_itr->y());
        BOOST_TEST(p->margin == margin);
      }
    }
  }

  { // add_robot_paths_if(...)
    // 結果
    const auto o = [&all, &pred, margin]() {
      using namespace std::chrono_literals;

      obstacle_list list;
      obstacle::add_robot_paths_if(list, all, margin, -500ms, 2400ms, pred);
      return list.buffer();
    }();

    // サイズをチェック
    BOOST_TEST(o.size() == robots.size());

    // 障害物リストの各要素をチェック
    auto r_itr = ref_robots.begin();
    auto o_itr = o.begin();
    for (; r_itr != ref_robots.end() && o_itr != o.end(); ++r_itr, ++o_itr) {
      BOOST_TEST_CONTEXT("at " << std::distance(ref_robots.begin(), r_itr)) {
        const auto p = std::get_if<obstacle::segment>(&o_itr->second);
        BOOST_TEST(p != nullptr);
        BOOST_TEST(std::get<0>(p->geometry).x() == r_itr->x() - 0.5 * r_itr->vx());
        BOOST_TEST(std::get<0>(p->geometry).y() == r_itr->y() - 0.5 * r_itr->vy());
        BOOST_TEST(std::get<1>(p->geometry).x() == r_itr->x() + 2.4 * r_itr->vx());
        BOOST_TEST(std::get<1>(p->geometry).y() == r_itr->y() + 2.4 * r_itr->vy());
        BOOST_TEST(p->margin == margin);
      }
    }
  }
}

BOOST_AUTO_TEST_SUITE_END()
