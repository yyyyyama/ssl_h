#ifndef AI_SERVER_MODEL_ROBOT_H
#define AI_SERVER_MODEL_ROBOT_H

namespace ai_server {
namespace model {

class robot {
private:
  int id_;
  double x_;
  double y_;
  double vx_;
  double vy_;
  double theta_;
  double omega_;

  robot(int, double);

public:
  robot(unsigned int id, double x, double y);

  int id() const;
  double x() const;
  double y() const;
  double vx() const;
  double vy() const;
  double theta() const;
  double omega() const;

  void set_x(double x);
  void set_y(double y);
  void set_vx(double vx);
  void set_vy(double vy);
  void set_theta(double theta);
  void set_omega(double omega);
};
}
}

#endif
