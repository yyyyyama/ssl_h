#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE world_model_test

#include <stdexcept>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/detail/confidence_comparator.h"
#include "ai_server/model/world.h"

#include "ssl-protos/vision/detection.pb.h"

BOOST_AUTO_TEST_SUITE(confidence_comparator)

BOOST_AUTO_TEST_CASE(ball) {
  ssl_protos::vision::Ball ball1;
  ball1.set_confidence(98.0f);
  ssl_protos::vision::Ball ball2;
  ball2.set_confidence(95.0f);

  // ball1.confidence() > ball2.confidence()
  BOOST_TEST(ai_server::model::detail::confidence_comparator(ball1, ball2));
}

BOOST_AUTO_TEST_CASE(robot) {
  ssl_protos::vision::Robot robot1;
  robot1.set_confidence(98.0f);
  ssl_protos::vision::Robot robot2;
  robot2.set_confidence(95.0f);

  // robot1.confidence() > robot2.confidence()
  BOOST_TEST(ai_server::model::detail::confidence_comparator(robot1, robot2));
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(world_model)

BOOST_AUTO_TEST_CASE(detection) {
  ai_server::model::world w{};

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
    rb1->set_y(20);
    rb1->set_confidence(94.0);

    auto rb3 = md->add_robots_blue();
    rb3->set_robot_id(3);
    rb3->set_x(30);
    rb3->set_y(40);
    rb3->set_confidence(95.0);

    auto ry5 = md->add_robots_yellow();
    ry5->set_robot_id(5);
    ry5->set_x(100);
    ry5->set_y(200);
    ry5->set_confidence(96.0);

    auto ry7 = md->add_robots_yellow();
    ry7->set_robot_id(7);
    ry7->set_x(300);
    ry7->set_y(400);
    ry7->set_confidence(97.0);

    w.update(p);
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
    BOOST_TEST(r.y() == 20);

    // ID3の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(3));
    BOOST_TEST(r.id() == 3);
    BOOST_TEST(r.x() == 30);
    BOOST_TEST(r.y() == 40);

    // 検出された黃ロボは2台
    BOOST_TEST(w.robots_yellow().size() == 2);

    // ID5の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(5));
    BOOST_TEST(r.id() == 5);
    BOOST_TEST(r.x() == 100);
    BOOST_TEST(r.y() == 200);

    // ID7の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(7));
    BOOST_TEST(r.id() == 7);
    BOOST_TEST(r.x() == 300);
    BOOST_TEST(r.y() == 400);

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
    rb1->set_x(50);
    rb1->set_y(60);
    rb1->set_confidence(99.0);

    auto rb2 = md->add_robots_blue();
    rb2->set_robot_id(2);
    rb2->set_x(70);
    rb2->set_y(80);
    rb2->set_confidence(98.0);

    auto ry6 = md->add_robots_yellow();
    ry6->set_robot_id(6);
    ry6->set_x(500);
    ry6->set_y(600);
    ry6->set_confidence(97.0);

    auto ry7 = md->add_robots_yellow();
    ry7->set_robot_id(7);
    ry7->set_x(700);
    ry7->set_y(800);
    ry7->set_confidence(96.0);

    w.update(p);
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
    BOOST_TEST(r.x() == 50);
    BOOST_TEST(r.y() == 60);

    // ID2の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(2));
    BOOST_TEST(r.id() == 2);
    BOOST_TEST(r.x() == 70);
    BOOST_TEST(r.y() == 80);

    // ID3の青ロボが存在, 値の変化なし
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(3));
    BOOST_TEST(r.id() == 3);
    BOOST_TEST(r.x() == 30);
    BOOST_TEST(r.y() == 40);

    // 検出された黃ロボは3台
    BOOST_TEST(w.robots_yellow().size() == 3);

    // ID5の青ロボが存在, 値の変化なし
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(5));
    BOOST_TEST(r.id() == 5);
    BOOST_TEST(r.x() == 100);
    BOOST_TEST(r.y() == 200);

    // ID6の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(6));
    BOOST_TEST(r.id() == 6);
    BOOST_TEST(r.x() == 500);
    BOOST_TEST(r.y() == 600);

    // cam0で検出されたID7の黃ロボのほうがconfidenceが高いので値の変化なし
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(7));
    BOOST_TEST(r.id() == 7);
    BOOST_TEST(r.x() == 300);
    BOOST_TEST(r.y() == 400);
  }

  {
    ssl_protos::vision::Packet p;

    auto md = p.mutable_detection();
    md->set_camera_id(0);

    w.update(p);
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
    BOOST_TEST(r.x() == 50);
    BOOST_TEST(r.y() == 60);

    // cam1で検出されたID2の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_blue().at(2));
    BOOST_TEST(r.id() == 2);
    BOOST_TEST(r.x() == 70);
    BOOST_TEST(r.y() == 80);

    // ID3の青ロボが存在しない (ID3の要素を参照するとstd::out_of_range例外が飛ぶ)
    BOOST_CHECK_THROW(r = w.robots_blue().at(3), std::out_of_range);

    // 検出された黃ロボは2台
    BOOST_TEST(w.robots_yellow().size() == 2);

    // ID5の青ロボが存在しない
    BOOST_CHECK_THROW(r = w.robots_yellow().at(5), std::out_of_range);

    // ID6の青ロボが存在
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(6));
    BOOST_TEST(r.id() == 6);
    BOOST_TEST(r.x() == 500);
    BOOST_TEST(r.y() == 600);

    // cam0で検出されていたID7の黃ロボが消えたので,
    // cam1で検出された黄ロボ(700, 800)の値が出てくる
    BOOST_CHECK_NO_THROW(r = w.robots_yellow().at(7));
    BOOST_TEST(r.id() == 7);
    BOOST_TEST(r.x() == 700);
    BOOST_TEST(r.y() == 800);
  }
}

BOOST_AUTO_TEST_SUITE_END()
