#ifndef AI_SERVER_FILTER_COORDINATE_TRANSFORMATION_H
#define AI_SERVER_FILTER_COORDINATE_TRANSFORMATION_H

#include <chrono>

#include "ai_server/model/ball.h"
#include "ai_server/model/robot.h"
#include "base.h"

namespace ai_server {
namespace filter {

template <class T>
class coordinate_transformation : public base<T> {
private:
  double x_;
  double y_;
  double theta_;

public:
  coordinate_transformation(double x, double y, double theta);
  void apply(T&, std::chrono::high_resolution_clock::time_point) override;
};

template <class T>
coordinate_transformation<T>::coordinate_transformation(double x, double y, double theta)
    : x_(x), y_(y), theta_(theta) {}

} // filter
} // ai_server

#endif
