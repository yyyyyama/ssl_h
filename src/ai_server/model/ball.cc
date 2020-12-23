#include "ball.h"
namespace ai_server {
namespace model {

ball::ball() : x_(0), y_(0), z_(0), is_lost_(true) {}

ball::ball(double x, double y, double z) : x_(x), y_(y), z_(z) {}

double ball::x() const {
  return x_;
}

double ball::y() const {
  return y_;
}

double ball::z() const {
  return z_;
}

double ball::vx() const {
  return vx_;
}

double ball::vy() const {
  return vy_;
}

double ball::ax() const {
  return ax_;
}

double ball::ay() const {
  return ay_;
}

bool ball::is_lost() const {
  return is_lost_;
}

void ball::set_x(double x) {
  x_ = x;
}

void ball::set_y(double y) {
  y_ = y;
}

void ball::set_z(double z) {
  z_ = z;
}

void ball::set_vx(double vx) {
  vx_ = vx;
}

void ball::set_vy(double vy) {
  vy_ = vy;
}

void ball::set_ax(double ax) {
  ax_ = ax;
}

void ball::set_ay(double ay) {
  ay_ = ay;
}

void ball::set_is_lost(bool is_lost) {
  is_lost_ = is_lost;
}

bool ball::has_estimator() const {
  return static_cast<bool>(estimator_);
}

void ball::set_estimator(ball::estimator_type f) {
  estimator_ = std::move(f);
}

void ball::clear_estimator() {
  estimator_ = nullptr;
}

std::optional<ball> ball::state_after(std::chrono::system_clock::duration t) const {
  if (estimator_) {
    return estimator_(*this, t);
  } else {
    return std::nullopt;
  }
}

} // namespace model
} // namespace ai_server
