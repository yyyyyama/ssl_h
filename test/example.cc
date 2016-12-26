#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE field_data_test 

#include <boost/test/unit_test.hpp>

#include"ai_server/model/field.h"

BOOST_AUTO_TEST_SUITE(field_data)

BOOST_AUTO_TEST_CASE(test001) {
	ai_server::model::field f{100,90,80,70,60,50};

	BOOST_TEST(f.length()==100);
	BOOST_TEST(f.width()==90);
	BOOST_TEST(f.center_radius()==80);
	BOOST_TEST(f.goal_width()==70);
	BOOST_TEST(f.penalty_radius()==60);
	BOOST_TEST(f.penalty_line_length()==50);
}

BOOST_AUTO_TEST_CASE(test002) {
	ai_server::model::field f{};

	f.set_length(1);
	f.set_width(2);
	f.set_center_radius(3);
	f.set_goal_width(4);
	f.set_penalty_radius(5);
	f.set_penalty_line_length(6);
	BOOST_TEST(f.length()==1);
	BOOST_TEST(f.width()==2);
	BOOST_TEST(f.center_radius()==3);
	BOOST_TEST(f.goal_width()==4);
	BOOST_TEST(f.penalty_radius()==5);
	BOOST_TEST(f.penalty_line_length()==6);
}

BOOST_AUTO_TEST_CASE(test003) {
	ai_server::model::field f{100,90,80,70,60,50};

	BOOST_TEST(f.x_max()==50);
	BOOST_TEST(f.x_min()==-50);
	BOOST_TEST(f.y_max()==45);
	BOOST_TEST(f.y_min()==-45);
}

BOOST_AUTO_TEST_SUITE_END()
