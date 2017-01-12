#include "command.h"

namespace ai_server {
namespace model {

command::command(unsigned int id) : setpoint_(position_t{0.0, 0.0, 0.0}) {
  id_      = id;
  kick_    = 0;
  chip_    = 0;
  dribble_ = 0;
}

unsigned int command::id() const {
  return id_;
}

int command::kick() const {
  return kick_;
}

int command::chip() const {
  return chip_;
}

int command::dribble() const {
  return dribble_;
}

const setpoint_t& command::setpoint() const {
  return setpoint_;
}

void command::set_kick(int kick) {
  kick_ = kick;
}

void command::set_chip(int chip) {
  chip_ = chip;
}

void command::set_dribble(int dribble) {
  dribble_ = dribble;
}

void command::set_position(const position_t& position) {
  setpoint_ = position;
}

void command::set_velocity(const velocity_t& velocity) {
  setpoint_ = velocity;
}
}
}
