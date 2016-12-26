#include "ai_server/model/field.h"

namespace ai_server {
namespace model {
field::field(int length, int width, int center_radius, int goal_width, int penalty_radius,
             int penalty_line_length)
    : length_(length),
      width_(width),
      center_radius_(center_radius),
      goal_width_(goal_width),
      penalty_radius_(penalty_radius),
      penalty_line_length_(penalty_line_length) {}
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
int field::penalty_radius() const {
  return penalty_radius_;
}
int field::penalty_line_length() const {
  return penalty_line_length_;
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
void field::set_penalty_radius(int penalty_radius) {
  penalty_radius_ = penalty_radius;
}
void field::set_penalty_line_length(int penalty_line_length) {
  penalty_line_length_ = penalty_line_length;
}
int field::x_max() {
  return (length_ / 2);
}
int field::y_max() {
  return (width_ / 2);
}
int field::x_min() {
  return (-length_ / 2);
}
int field::y_min() {
  return (-width_ / 2);
}
}
}
