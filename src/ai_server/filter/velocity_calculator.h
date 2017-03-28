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
  T prev_state_;

public:
  velocity_calculator();
  void apply(T&, std::chrono::high_resolution_clock::time_point) override;
  void reset() override;
};

template <class T>
velocity_calculator<T>::velocity_calculator() : prev_state_(T{}) {}

template <class T>
void velocity_calculator<T>::reset() {
  prev_state_ = T{};
}
}
}

#endif