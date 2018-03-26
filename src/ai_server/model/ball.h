#ifndef AI_SERVER_MODEL_BALL_H
#define AI_SERVER_MODEL_BALL_H
namespace ai_server {
namespace model {

class ball {
private:
  double x_;
  double y_;
  double z_;
  double vx_;
  double vy_;
  double ax_;
  double ay_;

public:
  ball();
  ball(double x, double y, double z);
  double x() const;
  double y() const;
  double z() const;
  double vx() const;
  double vy() const;
  double ax() const;
  double ay() const;

  void set_x(double x);
  void set_y(double y);
  void set_z(double z);
  void set_vx(double vx);
  void set_vy(double vy);
  void set_ax(double ax);
  void set_ay(double ay);
};
} // namespace model
} // namespace ai_server
#endif
