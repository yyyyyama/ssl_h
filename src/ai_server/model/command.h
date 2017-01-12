#ifndef AI_SERVER_MODEL_COMMAND_H
#define AI_SERVER_MODEL_COMMAND_H
#include "boost/variant.hpp"

namespace ai_server {
namespace model {

struct position_t {
  double x;
  double y;
  double theta;
};

struct velocity_t {
  double vx;
  double vy;
  double omega;
};

using setpoint_t = boost::variant<position_t, velocity_t>;

class command {
public:
  explicit command(unsigned int id);

  unsigned int id() const;
  int kick() const;
  int chip() const;
  int dribble() const;
  const setpoint_t& setpoint() const;

  void set_kick(int kick);
  void set_chip(int chip);
  void set_dribble(int dribble);
  void set_position(const position_t& position);
  void set_velocity(const velocity_t& velocity);

private:
  unsigned int id_;
  int kick_;
  int chip_;
  int dribble_;
  setpoint_t setpoint_;
};
}
}

#endif
