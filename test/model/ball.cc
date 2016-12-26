#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE ball_datatype_test

#include<boost/test/unit_test.hpp>

#include"ai_server/model/ball.h"

BOOST_AUTO_TEST_SUITE(ball_datatype)

BOOST_AUTO_TEST_CASE(test01){
	//constructor
	ai_server::model::ball b{1, 2, 3};
	
	//get value
	BOOST_TEST(b.x() ==  1);
	BOOST_TEST(b.y() ==  2);
	BOOST_TEST(b.z() ==  3);
}

BOOST_AUTO_TEST_CASE(test02){
	ai_server::model::ball b;

	//set x
	b.set_x(4);
	BOOST_TEST(b.x() ==  4);
	//set y
	b.set_y(5);
	BOOST_TEST(b.y() ==  5);
	//set z
	b.set_z(6);
	BOOST_TEST(b.z() ==  6);
}

BOOST_AUTO_TEST_SUITE_END()

