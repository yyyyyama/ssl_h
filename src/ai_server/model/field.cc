#include "ai_server/model/field.h"

namespace ai_server {
namespace model {
field::field()
    : length_(4050),
      width_(3025),
      center_radius_(500),
      goal_width_(1000),
      penalty_length_(600),
      penalty_width_(1600) {}
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
  return 0.5 * length_;
}
double field::y_max() const {
  return 0.5 * width_;
}
double field::x_min() const {
  return -0.5 * length_;
}
double field::y_min() const {
  return -0.5 * width_;
}
double field::back_penalty_x() const {
  return x_min() + penalty_length_;
}
double field::front_penalty_x() const {
  return x_max() - penalty_length_;
}
double field::penalty_y_max() const {
  return 0.5 * penalty_width_;
}
double field::penalty_y_min() const {
  return -0.5 * penalty_width_;
}
double field::goal_y_max() const {
  return 0.5 * goal_width_;
}
double field::goal_y_min() const {
  return -0.5 * goal_width_;
}
field::point field::back_goal_center() const {
  return point{x_min(), 0.0};
}
field::point field::back_goal_left() const {
  return point{x_min(), goal_y_max()};
}
field::point field::back_goal_right() const {
  return point{x_min(), goal_y_min()};
}
field::point field::front_goal_center() const {
  return point{x_max(), 0.0};
}
field::point field::front_goal_left() const {
  return point{x_max(), goal_y_max()};
}
field::point field::front_goal_right() const {
  return point{x_max(), goal_y_min()};
}
field::point field::back_left_corner() const {
  return point{x_min(), y_max()};
}
field::point field::back_right_corner() const {
  return point{x_min(), y_min()};
}
field::point field::front_left_corner() const {
  return point{x_max(), y_max()};
}
field::point field::front_right_corner() const {
  return point{x_max(), y_min()};
}
field::point field::back_penalty_mark() const {
  return {x_max() - 700.0, 0.0};
}
field::point field::front_penalty_mark() const {
  return {x_min() + 700.0, 0.0};
}
field::box field::back_penalty_area() const {
  return box{point{x_min(), penalty_y_min()}, point{back_penalty_x(), penalty_y_max()}};
}
field::box field::front_penalty_area() const {
  return box{point{front_penalty_x(), penalty_y_min()}, point{x_max(), penalty_y_max()}};
}
field::box field::back_half_area() const {
  return box{point{x_min(), y_min()}, point{0.0, y_max()}};
}
field::box field::front_half_area() const {
  return box{point{0.0, y_min()}, point{x_max(), y_max()}};
}
field::box field::game_area() const {
  return box{point{x_min(), y_min()}, point{x_max(), y_max()}};
}

} // namespace model
} // namespace ai_server
