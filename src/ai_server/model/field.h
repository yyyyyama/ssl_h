#ifndef AI_SERVER_MODEL_FIELD_H
#define AI_SERVER_MODEL_FIELD_H

#include "geometry/box.h"
#include "geometry/point.h"

namespace ai_server {
namespace model {
class field {
private:
  int length_;
  int width_;
  int center_radius_;
  int goal_width_;
  int penalty_length_;
  int penalty_width_;

public:
  using box   = geometry::box<double>;
  using point = geometry::point<double>;

  field();
  int length() const;
  int width() const;
  int center_radius() const;
  int goal_width() const;
  int penalty_length() const;
  int penalty_width() const;
  void set_length(int length);
  void set_width(int width);
  void set_center_radius(int center_radius);
  void set_goal_width(int goal_width);
  void set_penalty_length(int penalty_length);
  void set_penalty_width(int penalty_width);
  double x_max() const;
  double x_min() const;
  double y_max() const;
  double y_min() const;
  // ペナルティエリア
  double back_penalty_x() const;
  double front_penalty_x() const;
  double penalty_y_max() const;
  double penalty_y_min() const;
  // ゴール
  double goal_y_max() const;
  double goal_y_min() const;
  // 後方ゴール
  point back_goal_center() const;
  point back_goal_left() const;
  point back_goal_right() const;
  // 前方ゴール
  point front_goal_center() const;
  point front_goal_left() const;
  point front_goal_right() const;
  // 後方コーナー
  point back_left_corner() const;
  point back_right_corner() const;
  // 前方コーナー
  point front_left_corner() const;
  point front_right_corner() const;
  // ペナルティマーク
  point back_penalty_mark() const;
  point front_penalty_mark() const;
  // ハーフエリア
  box back_half_area() const;
  box front_half_area() const;
  // ペナルティエリア
  box back_penalty_area() const;
  box front_penalty_area() const;
  // 白線の内側
  box game_area() const;
};
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_FIELD_H
