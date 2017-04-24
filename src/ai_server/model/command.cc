#include "command.h"

namespace ai_server {
namespace model {

command::command(unsigned int id)
    : id_(id),
      dribble_(0),
      kick_flag_(command::kick_type_t::none, 0.0),
      setpoint_(command::velocity_t{0.0, 0.0, 0.0}) {}

unsigned int command::id() const {
  return id_;
}

int command::dribble() const {
  return dribble_;
}

command::kick_flag_t command::kick_flag() const {
  return kick_flag_;
}

const command::setpoint_t& command::setpoint() const {
  return setpoint_;
}

void command::set_dribble(int dribble) {
  dribble_ = dribble;
}

void command::set_kick_flag(const command::kick_flag_t& kick_flag) {
  kick_flag_ = kick_flag;
}

void command::set_position(const command::position_t& position) {
  setpoint_ = position;
}

void command::set_velocity(const command::velocity_t& velocity) {
  setpoint_ = velocity;
}

void command::set_acceleration(const command::acceleration_t& accelerationi){
  setpoint_=acceleration;
}
}
}
