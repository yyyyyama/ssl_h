#ifndef AI_SERVER_MODEL_COMMAND_H
#define AI_SERVER_MODEL_COMMAND_H
#include <memory>
#include <tuple>
#include <utility>
#include <variant>

#include <Eigen/Core>

#include "ai_server/model/motion/base.h"
#include "ai_server/model/setpoint/types.h"
#include "ai_server/util/dependent_false.h"

namespace ai_server {
namespace model {

class command {
public:
  enum class kick_type_t { none, line, chip, backspin };

  using kick_flag_t = std::tuple<kick_type_t, double>;

  command();

  int dribble() const;
  kick_flag_t kick_flag() const;

  void set_dribble(int dribble);
  void set_kick_flag(const kick_flag_t& kick_flag);

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

  void set_position(double x, double y, double theta) {
    set_position(x, y);
    set_angle(theta);
  }

  void set_velocity(double vx, double vy, double omega) {
    set_velocity(vx, vy);
    set_velanglar(omega);
  }

  // set_position({x, y, theta}), set_velocity({vx, vy, omega}) といった記述向けのオーバーロード
  void set_position(const std::tuple<double, double, double>& p) {
    set_position(std::get<0>(p), std::get<1>(p), std::get<2>(p));
  }

  void set_velocity(const std::tuple<double, double, double>& v) {
    set_velocity(std::get<0>(v), std::get<1>(v), std::get<2>(v));
  }

  // Eigen のベクトルを取る set_{position,velocity} のオーバーロード
  // Eigen::Vector2d などをそのまま引数にすると set_position({x, y, theta}) などの
  // オーバーロード解決に失敗するためこのような実装とした
#define AI_SERVER_MODEL_COMMAND_OVERLOAD_FOR_EIGEN(name)                                 \
  template <class Derived>                                                               \
  void name(const Eigen::MatrixBase<Derived>& v) {                                       \
    using matrix_type        = Eigen::MatrixBase<Derived>;                               \
    constexpr auto is_double = std::is_same_v<typename matrix_type::value_type, double>; \
    constexpr auto rows      = matrix_type::RowsAtCompileTime;                           \
    constexpr auto cols      = matrix_type::ColsAtCompileTime;                           \
    if constexpr (is_double && rows == 3 && cols == 1) {                                 \
      name(v.x(), v.y(), v.z());                                                         \
    } else if constexpr (is_double && rows == 2 && cols == 1) {                          \
      name(v.x(), v.y());                                                                \
    } else {                                                                             \
      static_assert(util::dependent_false_v<matrix_type>, "unsupported matrix type");    \
    }                                                                                    \
  }

  AI_SERVER_MODEL_COMMAND_OVERLOAD_FOR_EIGEN(set_position)
  AI_SERVER_MODEL_COMMAND_OVERLOAD_FOR_EIGEN(set_velocity)

#undef AI_SERVER_MODEL_COMMAND_OVERLOAD_FOR_EIGEN

  void set_position(const Eigen::Vector2d& p, double theta) {
    set_position(p.x(), p.y(), theta);
  }

  void set_velocity(const Eigen::Vector2d& v, double omega) {
    set_velocity(v.x(), v.y(), omega);
  }

  std::pair<const setpoint::position_or_velocity&, const setpoint::angle_or_velangular&>
  setpoint_pair() const {
    return {setpoint_, setpoint_rot_};
  }

  void set_motion(const std::shared_ptr<model::motion::base>& motion) {
    motion_ = motion;
  }

  std::shared_ptr<model::motion::base> motion() {
    return motion_;
  }

private:
  int dribble_;
  kick_flag_t kick_flag_;

  setpoint::position_or_velocity setpoint_;
  setpoint::angle_or_velangular setpoint_rot_;

  std::shared_ptr<model::motion::base> motion_;
};
} // namespace model
} // namespace ai_server

#endif
