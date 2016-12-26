#include "ball.h"
namespace ai_server {
namespace model {

ball::ball(double x = 0.0, double y = 0.0, double z = 0.0) : x_(x), y_(y), z_(z){};

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
}
}
