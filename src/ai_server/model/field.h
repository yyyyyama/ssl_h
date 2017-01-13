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
  int penalty_radius_;
  int penalty_line_length_;

public:
  int length() const;
  int width() const;
  int center_radius() const;
  int goal_width() const;
  int penalty_radius() const;
  int penalty_line_length() const;
  void set_length(int length);
  void set_width(int width);
  void set_center_radius(int center_radius);
  void set_goal_width(int goal_width);
  void set_penalty_radius(int penalty_radius);
  void set_penalty_line_length(int penalty_line_length);
  double x_max() const;
  double x_min() const;
  double y_max() const;
  double y_min() const;
};
}
}

#endif // AI_SERVER_MODEL_FIELD_H
