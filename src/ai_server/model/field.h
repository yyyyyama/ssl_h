#ifndef AI_SERVER_MODEL_FIELD_H
#define AI_SERVER_MODEL_FIELD_H

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
};
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_FIELD_H
