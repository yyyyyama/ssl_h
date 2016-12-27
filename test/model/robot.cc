#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE robotdata_Test

#include <boost/test/unit_test.hpp>
#include "src/ai_server/model/robot.h"


BOOST_AUTO_TEST_SUITE(robotdata)

BOOST_AUTO_TEST_CASE(test001) {
	
	// constructor test
  ai_server::model::robot rob{1,2,3};
 
  // get value test
  BOOST_TEST(rob.id() == 1);
  BOOST_TEST(rob.x() == 2);
  BOOST_TEST(rob.y() == 3);
  
  
  //set value test
  rob.set_x(10);
  BOOST_TEST(rob.x() == 10);
  
  rob.set_y(11);
  BOOST_TEST(rob.y() == 11);
  
    rob.set_vx(12);
  BOOST_TEST(rob.y() == 12);
  
    rob.set_vy(13);
  BOOST_TEST(rob.y() == 13);
  
    rob.set_theta(14);
  BOOST_TEST(rob.theta() == 14);
  
    rob.set_omega(15);
  BOOST_TEST(rob.omega() == 15);
}


BOOST_AUTO_TEST_CASE(test002) {
	
	// constructor test
  ai_server::model::robot rob2{21};
  
  
  // get value test
  BOOST_TEST(rob2.id() == 21);
  BOOST_TEST(rob2.x() == 0);
  BOOST_TEST(rob2.y() == 0);
  
  
  //set value test
  rob2.set_x(40);
  BOOST_TEST(rob2.x() == 40);
  
  rob2.set_y(41);
  BOOST_TEST(rob2.y() == 41);
  
    rob2.set_vx(42);
  BOOST_TEST(rob2.y() == 42);
  
    rob2.set_vy(43);
  BOOST_TEST(rob2.y() == 43);
  
    rob2.set_theta(44);
  BOOST_TEST(rob2.theta() == 44);
  
    rob2.set_omega(45);
  BOOST_TEST(rob2.omega() == 45);
}

BOOST_AUTO_TEST_SUITE_END()
