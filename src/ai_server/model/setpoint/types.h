#ifndef AI_SERVER_MODEL_SETPOINT_TYPES_H
#define AI_SERVER_MODEL_SETPOINT_TYPES_H

#include <tuple>
#include <variant>

namespace ai_server::model::setpoint {

namespace phantom {
struct position {};
struct velocity {};
struct angle {};
struct velangular {};
} // namespace phantom

/// 座標 (x [mm], y [mm])
using position = std::tuple<double, double, phantom::position>;

/// 速度 (vx [mm/s], vy [mm/s])
using velocity = std::tuple<double, double, phantom::velocity>;

/// 角度 (theta [rad])
using angle = std::tuple<double, phantom::angle>;

/// 角速度 (omega [rad/s])
using velangular = std::tuple<double, phantom::velangular>;

/// 座標または速度
using position_or_velocity = std::variant<position, velocity>;

/// 角度または角速度
using angle_or_velangular = std::variant<angle, velangular>;

} // namespace ai_server::model::setpoint

#endif // AI_SERVER_MODEL_SETPOINT_TYPES_H
