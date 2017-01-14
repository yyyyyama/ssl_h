#ifndef AI_SERVER_MODEL_COMMAND_H
#define AI_SERVER_MODEL_COMMAND_H
#include <tuple>
#include <boost/variant.hpp>

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

  using setpoint_t  = boost::variant<position_t, velocity_t>;
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

private:
  unsigned int id_;
  int dribble_;
  kick_flag_t kick_flag_;
  setpoint_t setpoint_;
};
}
}

#endif
