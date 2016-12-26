#include "ball.h"
namespace ai_server {
namespace model {

ball::ball(int x, int y, int z) : x_(x), y_(y), z_(z){};
ball::ball(){};

int ball::x() const {
  return x_;
}

int ball::y() const {
  return y_;
}

int ball::z() const {
  return z_;
}

int ball::vx() const {
  return vx_;
}

int ball::vy() const {
  return vy_;
}

int ball::ax() const {
  return ax_;
}

int ball::ay() const {
  return ay_;
}

void ball::set_x(int x) {
  x_ = x;
}

void ball::set_y(int y) {
  y_ = y;
}

void ball::set_z(int z) {
  z_ = z;
}

void ball::set_vx(int vx) {
  vx_ = vx;
}

void ball::set_vy(int vy) {
  vy_ = vy;
}

void ball::set_ax(int ax) {
  ax_ = ax;
}

void ball::set_ay(int ay) {
  ay_ = ay;
}
}
}
