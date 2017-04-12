#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE world_updater_test

#include <stdexcept>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/world_updater.h"

#include "ssl-protos/vision/detection.pb.h"

BOOST_AUTO_TEST_SUITE(world_updater)

BOOST_AUTO_TEST_CASE(detection) {
  ai_server::model::world_updater wu{};
  const auto& w = wu.world_model();

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
    rb1->set_orientation(12);
    rb1->set_confidence(94.0);

    auto rb3 = md->add_robots_blue();
    rb3->set_robot_id(3);
    rb3->set_x(30);
    rb3->set_y(31);
    rb3->set_orientation(32);
    rb3->set_confidence(95.0);

    auto ry5 = md->add_robots_yellow();
    ry5->set_robot_id(5);
    ry5->set_x(500);
    ry5->set_y(501);
    ry5->set_orientation(502);
    ry5->set_confidence(96.0);

    auto ry7 = md->add_robots_yellow();
    ry7->set_robot_id(7);
    ry7->set_x(700);
    ry7->set_y(701);
    ry7->set_orientation(702);
    ry7->set_confidence(97.0);

    wu.update(p);
  }

  {
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
    BOOST_TEST(r.theta() == 12);

    // ID3の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(3));
    BOOST_TEST(r.id() == 3);
    BOOST_TEST(r.x() == 30);
    BOOST_TEST(r.y() == 31);
    BOOST_TEST(r.theta() == 32);

    // 検出された黃ロボは2台
    BOOST_TEST(w.robots_yellow().size() == 2);

    // ID5の黃ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(5));
    BOOST_TEST(r.id() == 5);
    BOOST_TEST(r.x() == 500);
    BOOST_TEST(r.y() == 501);
    BOOST_TEST(r.theta() == 502);

    // ID7の黃ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(7));
    BOOST_TEST(r.id() == 7);
    BOOST_TEST(r.x() == 700);
    BOOST_TEST(r.y() == 701);
    BOOST_TEST(r.theta() == 702);

    // fieldは変更されていない (初期値のまま)
    const auto f = w.field();
    BOOST_TEST(f.length() == 0);
    BOOST_TEST(f.width() == 0);
    BOOST_TEST(f.center_radius() == 0);
    BOOST_TEST(f.goal_width() == 0);
    BOOST_TEST(f.penalty_radius() == 0);
    BOOST_TEST(f.penalty_line_length() == 0);
  }

  {
    ssl_protos::vision::Packet p;

    auto md = p.mutable_detection();
    md->set_camera_id(1);

    auto b = md->add_balls();
    b->set_x(4);
    b->set_y(5);
    b->set_z(6);
    b->set_confidence(100.0);

    auto rb1 = md->add_robots_blue();
    rb1->set_robot_id(1);
    rb1->set_x(13);
    rb1->set_y(14);
    rb1->set_orientation(15);
    rb1->set_confidence(99.0);

    auto rb2 = md->add_robots_blue();
    rb2->set_robot_id(2);
    rb2->set_x(20);
    rb2->set_y(21);
    rb2->set_orientation(22);
    rb2->set_confidence(98.0);

    auto ry6 = md->add_robots_yellow();
    ry6->set_robot_id(6);
    ry6->set_x(600);
    ry6->set_y(601);
    ry6->set_orientation(602);
    ry6->set_confidence(97.0);

    auto ry7 = md->add_robots_yellow();
    ry7->set_robot_id(7);
    ry7->set_x(703);
    ry7->set_y(704);
    ry7->set_orientation(705);
    ry7->set_confidence(96.0);

    wu.update(p);
  }

  {
    // cam1で検出されたボール(4, 5, 6)のほうがconfidenceが高い
    const auto b = w.ball();
    BOOST_TEST(b.x() == 4);
    BOOST_TEST(b.y() == 5);
    BOOST_TEST(b.z() == 6);

    ai_server::model::robot r;

    // 検出された青ロボは3台
    BOOST_TEST(w.robots_blue().size() == 3);

    // cam1で検出されたID1の青ロボ(50, 60)のほうがconfidenceが高い
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(1));
    BOOST_TEST(r.id() == 1);
    BOOST_TEST(r.x() == 13);
    BOOST_TEST(r.y() == 14);
    BOOST_TEST(r.theta() == 15);

    // ID2の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(2));
    BOOST_TEST(r.id() == 2);
    BOOST_TEST(r.x() == 20);
    BOOST_TEST(r.y() == 21);
    BOOST_TEST(r.theta() == 22);

    // ID3の青ロボが存在, 値の変化なし
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(3));
    BOOST_TEST(r.id() == 3);
    BOOST_TEST(r.x() == 30);
    BOOST_TEST(r.y() == 31);
    BOOST_TEST(r.theta() == 32);

    // 検出された黃ロボは3台
    BOOST_TEST(w.robots_yellow().size() == 3);

    // ID5の黃ロボが存在, 値の変化なし
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(5));
    BOOST_TEST(r.id() == 5);
    BOOST_TEST(r.x() == 500);
    BOOST_TEST(r.y() == 501);
    BOOST_TEST(r.theta() == 502);

    // ID6の黃ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(6));
    BOOST_TEST(r.id() == 6);
    BOOST_TEST(r.x() == 600);
    BOOST_TEST(r.y() == 601);
    BOOST_TEST(r.theta() == 602);

    // cam0で検出されたID7の黃ロボのほうがconfidenceが高いので値の変化なし
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(7));
    BOOST_TEST(r.id() == 7);
    BOOST_TEST(r.x() == 700);
    BOOST_TEST(r.y() == 701);
    BOOST_TEST(r.theta() == 702);
  }

  {
    ssl_protos::vision::Packet p;

    auto md = p.mutable_detection();
    md->set_camera_id(0);

    wu.update(p);
  }

  {
    // cam1で検出されたボールが存在
    const auto b = w.ball();
    BOOST_TEST(b.x() == 4);
    BOOST_TEST(b.y() == 5);
    BOOST_TEST(b.z() == 6);

    ai_server::model::robot r;

    // 検出された青ロボは2台
    BOOST_TEST(w.robots_blue().size() == 2);

    // cam1で検出されたID1の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(1));
    BOOST_TEST(r.id() == 1);
    BOOST_TEST(r.x() == 13);
    BOOST_TEST(r.y() == 14);
    BOOST_TEST(r.theta() == 15);

    // cam1で検出されたID2の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(2));
    BOOST_TEST(r.id() == 2);
    BOOST_TEST(r.x() == 20);
    BOOST_TEST(r.y() == 21);
    BOOST_TEST(r.theta() == 22);

    // ID3の青ロボが存在しない (ID3の要素を参照するとstd::out_of_range例外が飛ぶ)
    BOOST_CHECK_THROW(r = w.robots_blue().at(3), std::out_of_range);

    // 検出された黃ロボは2台
    BOOST_TEST(w.robots_yellow().size() == 2);

    // ID5の黃ロボが存在しない
    BOOST_CHECK_THROW(r = w.robots_yellow().at(5), std::out_of_range);

    // ID6の黃ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(6));
    BOOST_TEST(r.id() == 6);
    BOOST_TEST(r.x() == 600);
    BOOST_TEST(r.y() == 601);
    BOOST_TEST(r.theta() == 602);

    // cam0で検出されていたID7の黃ロボが消えたので,
    // cam1で検出された黄ロボ(700, 800)の値が出てくる
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(7));
    BOOST_TEST(r.id() == 7);
    BOOST_TEST(r.x() == 703);
    BOOST_TEST(r.y() == 704);
    BOOST_TEST(r.theta() == 705);
  }
}

BOOST_AUTO_TEST_CASE(detection_no_ball) {
  ai_server::model::world_updater wu{};
  const auto& w = wu.world_model();

  ssl_protos::vision::Packet p;

  auto md = p.mutable_detection();
  md->set_camera_id(0);

  BOOST_CHECK_NO_THROW(wu.update(p));
}

BOOST_AUTO_TEST_CASE(geometry) {
  ai_server::model::world_updater wu{};
  const auto& w = wu.world_model();

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

  {
    ssl_protos::vision::Packet p;

    auto mf = p.mutable_geometry()->mutable_field();
    mf->set_field_length(6000);
    mf->set_field_width(4500);
    mf->set_goal_width(800);

    auto cca = mf->add_field_arcs();
    cca->set_name("CenterCircle");
    cca->set_radius(100);

    auto lpa = mf->add_field_arcs();
    lpa->set_name("LeftFieldLeftPenaltyArc");
    lpa->set_radius(250);

    auto lpl = mf->add_field_lines();
    lpl->set_name("LeftPenaltyStretch");
    auto mp1 = lpl->mutable_p1();
    mp1->set_x(-2750);
    mp1->set_y(-250);
    auto mp2 = lpl->mutable_p2();
    mp2->set_x(-2750);
    mp2->set_y(250);

    wu.update(p);
  }

  {
    const auto f = w.field();
    BOOST_TEST(f.length() == 6000);
    BOOST_TEST(f.width() == 4500);
    BOOST_TEST(f.goal_width() == 800);
    BOOST_TEST(f.center_radius() == 100);
    BOOST_TEST(f.penalty_radius() == 250);
    BOOST_TEST(f.penalty_line_length() == 500);
  }
}

BOOST_AUTO_TEST_SUITE_END()
