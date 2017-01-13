#include "robot.h"

namespace ai_server {
namespace model {

robot::robot() : id_(0), x_(0), y_(0) {}

robot::robot(unsigned int id, double x, double y) : id_(id), x_(x), y_(y) {}

int robot::id() const {
  return id_;
}

double robot::x() const {
  return x_;
}

double robot::y() const {
  return y_;
}

double robot::vx() const {
  return vx_;
}

double robot::vy() const {
  return vy_;
}

double robot::theta() const {
  return theta_;
}

double robot::omega() const {
  return omega_;
}

void robot::set_x(double x) {
  x_ = x;
}

void robot::set_y(double y) {
  y_ = y;
}

void robot::set_vx(double vx) {
  vx_ = vx;
}

void robot::set_vy(double vy) {
  vy_ = vy;
}

void robot::set_theta(double theta) {
  theta_ = theta;
}

void robot::set_omega(double omega) {
  omega_ = omega;
}
}
}
