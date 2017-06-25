#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE updater_field_test

#include <stdexcept>
#include <boost/test/unit_test.hpp>

#include "ai_server/model/updater/field.h"
#include "ssl-protos/vision/geometry.pb.h"

BOOST_AUTO_TEST_CASE(field) {
  ai_server::model::updater::field fu;

  {
    // デフォルトコンストラクが呼ばれたときに, 内部の値がちゃんと初期化されているか
    const auto f1 = fu.value();
    const auto f2 = ai_server::model::field{};

    BOOST_TEST(f1.length() == f2.length());
    BOOST_TEST(f1.width() == f2.width());
    BOOST_TEST(f1.center_radius() == f2.center_radius());
    BOOST_TEST(f1.goal_width() == f2.goal_width());
    BOOST_TEST(f1.penalty_radius() == f2.penalty_radius());
    BOOST_TEST(f1.penalty_line_length() == f2.penalty_line_length());
  }

  {
    ssl_protos::vision::Geometry geometry;

    auto mf = geometry.mutable_field();
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

    fu.update(geometry);
  }

  {
    const auto f = fu.value();
    BOOST_TEST(f.length() == 9000);
    BOOST_TEST(f.width() == 6000);
    BOOST_TEST(f.goal_width() == 1000);
    BOOST_TEST(f.center_radius() == 200);
    BOOST_TEST(f.penalty_radius() == 500);
    BOOST_TEST(f.penalty_line_length() == 300);
  }

  {
    ssl_protos::vision::Geometry geometry;

    auto mf = geometry.mutable_field();
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

    fu.update(geometry);
  }

  {
    const auto f = fu.value();
    BOOST_TEST(f.length() == 6000);
    BOOST_TEST(f.width() == 4500);
    BOOST_TEST(f.goal_width() == 800);
    BOOST_TEST(f.center_radius() == 100);
    BOOST_TEST(f.penalty_radius() == 250);
    BOOST_TEST(f.penalty_line_length() == 500);
  }
}
