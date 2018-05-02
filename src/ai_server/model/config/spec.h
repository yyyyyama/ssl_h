#ifndef AI_SERVER_MODEL_CONFIG_SPEC_H
#define AI_SERVER_MODEL_CONFIG_SPEC_H

#include <optional>
#include <tuple>
#include <boost/hana/adapt_struct.hpp>

namespace ai_server::model::config {

struct spec {
  /// マシンの種類
  std::string type;
  /// 車輪の半径 [mm]
  double wheel_radius;
  /// ギア比
  double gear_ratio;
  /// 車体の半径 [mm]
  double machine_radius;
  /// 車輪の角度 [deg]
  std::array<double, 4> motor_angle;
  /// 最高速度 [mm/s]
  double max_velocity;
  /// 最高加速度 [mm/s^2]
  double max_acceleration;
  /// チップパワー変換係数 (飛距離➔送信パラメータ)
  std::optional<std::tuple<>> chip_power;
  /// キックパワー変換係数 (速度➔マシン送信パラメータ)
  std::optional<std::tuple<>> kick_power;
  /// バックスピンチップキックが可能か
  bool backspin;
};

} // namespace ai_server::model::config

BOOST_HANA_ADAPT_STRUCT(ai_server::model::config::spec, type, wheel_radius, gear_ratio,
                        machine_radius, motor_angle, max_velocity, max_acceleration, chip_power,
                        kick_power, backspin);

#endif // AI_SERVER_MODEL_CONFIG_SPEC_H
