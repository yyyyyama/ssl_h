#ifndef _AI_SERVER_MODEL_FIELD_H
#define _AI_SERVER_MODEL_FIELD_H

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
  field(int length, int width, int center_radius, int goal_width, int penalty_radius,
        int penalty_line_length);
  int length();
  int width();
  int center_radius();
  int goal_width();
  int penalty_radius();
  int penalty_line_length();
  void set_length(int length);
  void set_width(int width);
  void set_center_radius(int center_radius);
  void set_goal_width(int goal_width);
  void penalty_radius(int penalty_radius);
  void penalty_line_length(int penalty_line_length);
  int x_max();
  int x_min();
  int y_max();
  int y_min();
};
}
}

#endif //_AI_SERVER_MODEL_FIELD_H
