#include "robot.h"

namespace ai_server {
namespace model {

robot::robot() : x_(0), y_(0), theta_(0) {}

robot::robot(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {}

double robot::x() const {
  return x_;
}

double robot::y() const {
  return y_;
}

double robot::theta() const {
  return theta_;
}

double robot::vx() const {
  return vx_;
}

double robot::vy() const {
  return vy_;
}

double robot::omega() const {
  return omega_;
}

double robot::ax() const {
  return ax_;
}

double robot::ay() const {
  return ay_;
}

double robot::alpha() const {
  return alpha_;
}

void robot::set_x(double x) {
  x_ = x;
}

void robot::set_y(double y) {
  y_ = y;
}

void robot::set_theta(double theta) {
  theta_ = theta;
}

void robot::set_vx(double vx) {
  vx_ = vx;
}

void robot::set_vy(double vy) {
  vy_ = vy;
}

void robot::set_omega(double omega) {
  omega_ = omega;
}

void robot::set_ax(double ax) {
  ax_ = ax;
}

void robot::set_ay(double ay) {
  ay_ = ay;
}

void robot::set_alpha(double alpha) {
  alpha_ = alpha;
}

bool robot::has_estimator() const {
  return static_cast<bool>(estimator_);
}

void robot::set_estimator(robot::estimator_type f) {
  estimator_ = std::move(f);
}

void robot::clear_estimator() {
  estimator_ = nullptr;
}

std::optional<robot> robot::state_after(std::chrono::system_clock::duration t) const {
  if (estimator_) {
    return estimator_(*this, t);
  } else {
    return std::nullopt;
  }
}

} // namespace model
} // namespace ai_server
