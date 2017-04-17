#ifndef AI_SERVER_FILTER_COORDINATE_TRANSFORMATION_H
#define AI_SERVER_FILTER_COORDINATE_TRANSFORMATION_H

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
  void apply(T&, std::chrono::high_resolution_ckock::time_point) override;
};

template <>
void coordinate_transformation<model::ball>::apply(
    model::ball& ball, std::chrono::high_resolution_ckock::time_point);

template <>
void coordinate_transformation<model::robot>::apply(
    model::robot& robot, std::chrono::high_resolution_ckock::time_point);
} // filter
} // ai_server

#endif
