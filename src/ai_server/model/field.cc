#include "ai_server/model/field.h"

namespace ai_server {
namespace model {
field::field()
    : length_(12000),
      width_(9000),
      center_radius_(500),
      goal_width_(1200),
      penalty_length_(1200),
      penalty_width_(2400) {}
int field::length() const {
  return length_;
}
int field::width() const {
  return width_;
}
int field::center_radius() const {
  return center_radius_;
}
int field::goal_width() const {
  return goal_width_;
}
int field::penalty_length() const {
  return penalty_length_;
}
int field::penalty_width() const {
  return penalty_width_;
}
void field::set_length(int length) {
  length_ = length;
}
void field::set_width(int width) {
  width_ = width;
}
void field::set_center_radius(int center_radius) {
  center_radius_ = center_radius;
}
void field::set_goal_width(int goal_width) {
  goal_width_ = goal_width;
}
void field::set_penalty_length(int penalty_length) {
  penalty_length_ = penalty_length;
}
void field::set_penalty_width(int penalty_width) {
  penalty_width_ = penalty_width;
}
double field::x_max() const {
  return (length_ / 2);
}
double field::y_max() const {
  return (width_ / 2);
}
double field::x_min() const {
  return (-length_ / 2);
}
double field::y_min() const {
  return (-width_ / 2);
}
} // namespace model
} // namespace ai_server
