#ifndef AI_SERVER_MODEL_GEOMETRY_POINT_H
#define AI_SERVER_MODEL_GEOMETRY_POINT_H

#include <type_traits>
#include <utility>
#include <Eigen/Core>

namespace ai_server::model::geometry {

/// @struct   point
/// @brief    xy平面における位置座標を表現する型
template <class T>
struct point {
  T x;
  T y;

  /// @brief 任意の型に変換する
  /// @return {x, y}で初期化された値
  template <class U,
            std::enable_if_t<std::is_constructible_v<U, T, T>, std::nullptr_t> = nullptr>
  constexpr U as() const {
    return {x, y};
  }

  /// @brief std::pair<U, U>に変換する
  /// @return {x, y}で初期化された値
  template <class U = double>
  constexpr auto as_pair() const {
    return as<std::pair<U, U>>();
  }

  /// @brief Eigen::Matrix<U, 2, 1>に変換する
  /// @return {x, y}で初期化された値
  template <class U = double>
  inline auto as_vector() const {
    return as<Eigen::Matrix<U, 2, 1>>();
  }
};
} // namespace ai_server::model::geometry

#endif
