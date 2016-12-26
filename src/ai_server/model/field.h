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
  field(int length = 0, int width = 0, int center_radius = 0, int goal_width = 0,
        int penalty_radius = 0, int penalty_line_length = 0);
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
  int x_max();
  int x_min();
  int y_max();
  int y_min();
};
}
}

#endif //_AI_SERVER_MODEL_FIELD_H
