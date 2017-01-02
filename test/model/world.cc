#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE world_model_test

#include <boost/test/unit_test.hpp>

#include "ai_server/model/detail/confidence_comparator.h"

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
