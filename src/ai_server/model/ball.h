#ifndef AI_SERVER_MODEL_BALL_H
#define AI_SERVER_MODEL_BALL_H
namespace ai_server {
namespace model {

class ball {
private:
  int x_;
  int y_;
  int z_;
  int vx_;
  int vy_;
  int ax_;
  int ay_;

public:
  ball(int x, int y, int z);
  ball();
  int x() const;
  int y() const;
  int z() const;
  int vx() const;
  int vy() const;
  int ax() const;
  int ay() const;

  void set_x(int x);
  void set_y(int y);
  void set_z(int z);
  void set_vx(int vx);
  void set_vy(int vy);
  void set_ax(int ax);
  void set_ay(int ay);
};
}
}
#endif

