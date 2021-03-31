#include <stdexcept>

#include "command.h"

namespace ai_server {
namespace model {

command::command()
    : dribble_(0),
      kick_flag_(command::kick_type_t::none, 0.0),
      setpoint_(std::in_place_type<setpoint::velocity>, 0.0, 0.0,
                setpoint::phantom::velocity{}),
      setpoint_rot_(std::in_place_type<setpoint::velangular>, 0.0,
                    setpoint::phantom::velangular{}),
      motion_(nullptr) {}

int command::dribble() const {
  return dribble_;
}

command::kick_flag_t command::kick_flag() const {
  return kick_flag_;
}

void command::set_dribble(int dribble) {
  dribble_ = dribble;
}

void command::set_kick_flag(const command::kick_flag_t& kick_flag) {
  kick_flag_ = kick_flag;
}

} // namespace model
} // namespace ai_server
