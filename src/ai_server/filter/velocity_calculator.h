#ifndef AI_SERVER_FILTER_VELOCITY_CALCULATOR_H
#define AI_SERVER_FILTER_VELOCITY_CALCULATOR_H

#include "base.h"
#include "ai_server/model/ball.h"
#include "ai_server/model/robot.h"
#include <chrono>

namespace ai_server {
namespace filter {

template <class T>
class velocity_calculator : public base<T> {
private:
  std::chrono::high_resolution_clock::time_point prev_time_;
  ai_server::model::ball prev_ball_;
  ai_server::model::robot prev_robot_;

public:
  velocity_calculator();
  void apply(T&, std::chrono::high_resolution_clock::time_point) override;
  void reset() override; 

};
}
}

#endif

