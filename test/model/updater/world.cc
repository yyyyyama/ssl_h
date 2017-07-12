#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE updater_world_test

#include <stdexcept>
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/updater/world.h"
#include "ssl-protos/vision/wrapperpacket.pb.h"

BOOST_AUTO_TEST_SUITE(updater_world)

BOOST_AUTO_TEST_CASE(detection, *boost::unit_test::tolerance(0.0000001)) {
  namespace bmc = boost::math::double_constants;

  ai_server::model::updater::world wu{};

  {
    ssl_protos::vision::Packet p;

    auto md = p.mutable_detection();
    md->set_camera_id(0);

    auto b = md->add_balls();
    b->set_x(1);
    b->set_y(2);
    b->set_z(3);
    b->set_confidence(93.0);

    auto rb1 = md->add_robots_blue();
    rb1->set_robot_id(1);
    rb1->set_x(10);
    rb1->set_y(11);
    rb1->set_orientation(bmc::sixth_pi);
    rb1->set_confidence(94.0);

    auto rb3 = md->add_robots_blue();
    rb3->set_robot_id(3);
    rb3->set_x(30);
    rb3->set_y(31);
    rb3->set_orientation(bmc::third_pi);
    rb3->set_confidence(95.0);

    auto ry5 = md->add_robots_yellow();
    ry5->set_robot_id(5);
    ry5->set_x(500);
    ry5->set_y(501);
    ry5->set_orientation(bmc::half_pi);
    ry5->set_confidence(96.0);

    auto ry7 = md->add_robots_yellow();
    ry7->set_robot_id(7);
    ry7->set_x(700);
    ry7->set_y(701);
    ry7->set_orientation(bmc::two_thirds_pi);
    ry7->set_confidence(97.0);

    wu.update(p);
  }

  {
    const auto w = wu.value();

    // cam0で検出されたボール(1, 2, 3)
    const auto b = w.ball();
    BOOST_TEST(b.x() == 1);
    BOOST_TEST(b.y() == 2);
    BOOST_TEST(b.z() == 3);

    ai_server::model::robot r;

    // 検出された青ロボは2台
    BOOST_TEST(w.robots_blue().size() == 2);

    // ID1の青ロボが存在 (ID1の要素を参照してもstd::out_of_range例外が飛ばない)
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(1));
    BOOST_TEST(r.id() == 1);
    BOOST_TEST(r.x() == 10);
    BOOST_TEST(r.y() == 11);
    BOOST_TEST(r.theta() == bmc::sixth_pi);

    // ID3の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(3));
    BOOST_TEST(r.id() == 3);
    BOOST_TEST(r.x() == 30);
    BOOST_TEST(r.y() == 31);
    BOOST_TEST(r.theta() == bmc::third_pi);

    // 検出された黃ロボは2台
    BOOST_TEST(w.robots_yellow().size() == 2);

    // ID5の黃ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(5));
    BOOST_TEST(r.id() == 5);
    BOOST_TEST(r.x() == 500);
    BOOST_TEST(r.y() == 501);
    BOOST_TEST(r.theta() == bmc::half_pi);

    // ID7の黃ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(7));
    BOOST_TEST(r.id() == 7);
    BOOST_TEST(r.x() == 700);
    BOOST_TEST(r.y() == 701);
    BOOST_TEST(r.theta() == bmc::two_thirds_pi);

    // fieldは変更されていない (初期値のまま)
    const auto f = w.field();
    BOOST_TEST(f.length() == 0);
    BOOST_TEST(f.width() == 0);
    BOOST_TEST(f.center_radius() == 0);
    BOOST_TEST(f.goal_width() == 0);
    BOOST_TEST(f.penalty_radius() == 0);
    BOOST_TEST(f.penalty_line_length() == 0);
  }
}

BOOST_AUTO_TEST_CASE(transformation, *boost::unit_test::tolerance(0.0000001)) {
  namespace bmc = boost::math::double_constants;

  ai_server::model::updater::world wu{};
  // 90度回転, x軸方向に10, y軸方向に20平行移動
  wu.set_transformation_matrix(10, 20, bmc::half_pi);

  {
    ssl_protos::vision::Packet p;

    auto md = p.mutable_detection();
    md->set_camera_id(0);

    auto b = md->add_balls();
    b->set_x(1);
    b->set_y(2);
    b->set_z(3);
    b->set_confidence(93.0);

    auto rb1 = md->add_robots_blue();
    rb1->set_robot_id(1);
    rb1->set_x(10);
    rb1->set_y(11);
    rb1->set_orientation(bmc::sixth_pi);
    rb1->set_confidence(94.0);

    auto ry5 = md->add_robots_yellow();
    ry5->set_robot_id(5);
    ry5->set_x(500);
    ry5->set_y(501);
    ry5->set_orientation(bmc::pi);
    ry5->set_confidence(96.0);

    wu.update(p);
  }

  {
    const auto w = wu.value();

    // cam0で検出されたボール(1, 2, 3)
    const auto b = w.ball();
    BOOST_TEST(b.x() == 8.0);
    BOOST_TEST(b.y() == 21.0);
    BOOST_TEST(b.z() == 3);

    ai_server::model::robot r;

    // 検出された青ロボは1台
    BOOST_TEST(w.robots_blue().size() == 1);

    // ID1の青ロボが存在 (ID1の要素を参照してもstd::out_of_range例外が飛ばない)
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(1));
    BOOST_TEST(r.id() == 1);
    BOOST_TEST(r.x() == -1.0);
    BOOST_TEST(r.y() == 30.0);
    BOOST_TEST(r.theta() == bmc::pi + bmc::two_thirds_pi);

    // 検出された黃ロボは1台
    BOOST_TEST(w.robots_yellow().size() == 1);

    // ID5の黃ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(5));
    BOOST_TEST(r.id() == 5);
    BOOST_TEST(r.x() == -491.0);
    BOOST_TEST(r.y() == 520.0);
    BOOST_TEST(r.theta() == bmc::half_pi);
  }
}

BOOST_AUTO_TEST_CASE(geometry) {
  ai_server::model::updater::world wu{};

  {
    ssl_protos::vision::Packet p;

    auto mf = p.mutable_geometry()->mutable_field();
    mf->set_field_length(9000);
    mf->set_field_width(6000);
    mf->set_goal_width(1000);

    auto cca = mf->add_field_arcs();
    cca->set_name("CenterCircle");
    cca->set_radius(200);

    auto lpa = mf->add_field_arcs();
    lpa->set_name("LeftFieldLeftPenaltyArc");
    lpa->set_radius(500);

    auto lpl = mf->add_field_lines();
    lpl->set_name("LeftPenaltyStretch");
    auto mp1 = lpl->mutable_p1();
    mp1->set_x(-4000);
    mp1->set_y(-150);
    auto mp2 = lpl->mutable_p2();
    mp2->set_x(-4000);
    mp2->set_y(150);

    wu.update(p);
  }

  {
    const auto w = wu.value();

    const auto f = w.field();
    BOOST_TEST(f.length() == 9000);
    BOOST_TEST(f.width() == 6000);
    BOOST_TEST(f.goal_width() == 1000);
    BOOST_TEST(f.center_radius() == 200);
    BOOST_TEST(f.penalty_radius() == 500);
    BOOST_TEST(f.penalty_line_length() == 300);

    // ボールやロボットは検出されていない
    const auto b = w.ball();
    BOOST_TEST(b.x() == 0);
    BOOST_TEST(b.y() == 0);
    BOOST_TEST(b.z() == 0);
    BOOST_TEST(w.robots_blue().size() == 0);
    BOOST_TEST(w.robots_yellow().size() == 0);
  }
}

BOOST_AUTO_TEST_SUITE_END()