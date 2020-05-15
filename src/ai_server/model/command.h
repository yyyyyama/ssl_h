#ifndef AI_SERVER_MODEL_COMMAND_H
#define AI_SERVER_MODEL_COMMAND_H
#include <tuple>
#include <utility>
#include <variant>

#include "ai_server/model/setpoint/types.h"

namespace ai_server {
namespace model {

class command {
public:
  enum class kick_type_t { none, line, chip, backspin };

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

  struct acceleration_t {
    double ax;
    double ay;
    double alpha;
  };

  using setpoint_t  = std::variant<position_t, velocity_t>;
  using kick_flag_t = std::tuple<kick_type_t, double>;

  explicit command(unsigned int id);

  unsigned int id() const;
  int dribble() const;
  kick_flag_t kick_flag() const;
  const setpoint_t& setpoint() const;

  void set_dribble(int dribble);
  void set_kick_flag(const kick_flag_t& kick_flag);
  void set_position(const position_t& position);
  void set_velocity(const velocity_t& velocity);

  void set_position(double x, double y) {
    setpoint_.emplace<setpoint::position>(x, y, setpoint::phantom::position{});
  }

  void set_velocity(double vx, double vy) {
    setpoint_.emplace<setpoint::velocity>(vx, vy, setpoint::phantom::velocity{});
  }

  void set_angle(double theta) {
    setpoint_rot_.emplace<setpoint::angle>(theta, setpoint::phantom::angle{});
  }

  void set_velanglar(double omega) {
    setpoint_rot_.emplace<setpoint::velangular>(omega, setpoint::phantom::velangular{});
  }

  std::pair<const setpoint::position_or_velocity&, const setpoint::angle_or_velangular&>
  setpoint_pair() const {
    return {setpoint_, setpoint_rot_};
  }

private:
  unsigned int id_;
  int dribble_;
  kick_flag_t kick_flag_;

  setpoint::position_or_velocity setpoint_;
  setpoint::angle_or_velangular setpoint_rot_;

  // setpoint() の参照を使っている箇所へ対応するため
  mutable setpoint_t setpoint_old_;
};
} // namespace model
} // namespace ai_server

#endif
