#include <stdexcept>

#include "command.h"

namespace ai_server {
namespace model {

command::command(unsigned int id)
    : id_(id),
      dribble_(0),
      kick_flag_(command::kick_type_t::none, 0.0),
      setpoint_(std::in_place_type<setpoint::velocity>, 0.0, 0.0,
                setpoint::phantom::velocity{}),
      setpoint_rot_(std::in_place_type<setpoint::velangular>, 0.0,
                    setpoint::phantom::velangular{}) {}

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
  setpoint_old_ = std::visit(
      [](auto&& sp, auto&& sp_rot) -> command::setpoint_t {
        using sp_type     = std::decay_t<decltype(sp)>;
        using sp_rot_type = std::decay_t<decltype(sp_rot)>;

        [[maybe_unused]] constexpr auto sp_is_position =
            std::is_same_v<sp_type, setpoint::position>;
        [[maybe_unused]] constexpr auto sp_is_velocity =
            std::is_same_v<sp_type, setpoint::velocity>;

        [[maybe_unused]] constexpr auto sp_rot_is_angle =
            std::is_same_v<sp_rot_type, setpoint::angle>;
        [[maybe_unused]] constexpr auto sp_rot_is_velangular =
            std::is_same_v<sp_rot_type, setpoint::velangular>;

        if constexpr (sp_is_position && sp_rot_is_angle) {
          return position_t{std::get<0>(sp), std::get<1>(sp), std::get<0>(sp_rot)};
        } else if constexpr (sp_is_velocity && sp_rot_is_velangular) {
          return velocity_t{std::get<0>(sp), std::get<1>(sp), std::get<0>(sp_rot)};
        } else if constexpr (sp_is_position && sp_rot_is_velangular) {
          throw std::runtime_error{"position + velangular"};
        } else if constexpr (sp_is_velocity && sp_rot_is_angle) {
          throw std::runtime_error{"velocity + angle"};
        } else {
          static_assert(
              decltype(std::void_t<sp_type, sp_rot_type>(), std::false_type())::value);
        }
      },
      setpoint_, setpoint_rot_);
  return setpoint_old_;
}

void command::set_dribble(int dribble) {
  dribble_ = dribble;
}

void command::set_kick_flag(const command::kick_flag_t& kick_flag) {
  kick_flag_ = kick_flag;
}

void command::set_position(const command::position_t& position) {
  setpoint_.emplace<setpoint::position>(position.x, position.y, setpoint::phantom::position{});
  setpoint_rot_.emplace<setpoint::angle>(position.theta, setpoint::phantom::angle{});
}

void command::set_velocity(const command::velocity_t& velocity) {
  setpoint_.emplace<setpoint::velocity>(velocity.vx, velocity.vy,
                                        setpoint::phantom::velocity{});
  setpoint_rot_.emplace<setpoint::velangular>(velocity.omega, setpoint::phantom::velangular{});
}
} // namespace model
} // namespace ai_server
