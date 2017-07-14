#include "ai_server/model/field.h"

namespace ai_server {
namespace model {
field::field(){
  length_ = 9000;
  width_ = 6000;
  center_radius_ = 500;
  goal_width_ = 1000;
  penalty_radius_ = 1000;
  penalty_line_length_ = 500;	
}
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
}
}
