#ifndef AI_SERVER_UTIL_MATH_TO_VECTOR_H
#define AI_SERVER_UTIL_MATH_TO_VECTOR_H

#include <type_traits>

#include <Eigen/Dense>

#include "ai_server/model/command.h"

namespace ai_server {
namespace util {
namespace math {

/// @brief     メンバ関数x(), y()を持つオブジェクトを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 2, 1>{obj.x(), obj.y()}
template <class T>
inline auto position(const T& obj)
    -> Eigen::Matrix<std::common_type_t<decltype(obj.x()), decltype(obj.y())>, 2, 1> {
  return {obj.x(), obj.y()};
}

/// @brief     position_tを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.x, obj.y}
inline Eigen::Vector2d position(const model::command::position_t& obj) {
  return {obj.x, obj.y};
}

/// @brief     メンバ関数x(), y(), theta()を持つオブジェクトを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 3, 1>{obj.x(), obj.y(), obj.theta()}
template <class T>
inline auto position3d(const T& obj) -> Eigen::Matrix<
    std::common_type_t<decltype(obj.x()), decltype(obj.y()), decltype(obj.theta())>, 3, 1> {
  return {obj.x(), obj.y(), obj.theta()};
}

/// @brief     position_tを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.x, obj.y, obj.theta}
inline Eigen::Vector3d position3d(const model::command::position_t& obj) {
  return {obj.x, obj.y, obj.theta};
}

/// @brief     メンバ関数vx(), vy()を持つオブジェクトを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 2, 1>{obj.vx(), obj.vy()}
template <class T>
inline auto velocity(const T& obj)
    -> Eigen::Matrix<std::common_type_t<decltype(obj.vx()), decltype(obj.vy())>, 2, 1> {
  return {obj.vx(), obj.vy()};
}

/// @brief     velocity_tを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.vx, obj.vy}
inline Eigen::Vector2d velocity(const model::command::velocity_t& obj) {
  return {obj.vx, obj.vy};
}

/// @brief     メンバ関数vx(), vy(), omega()を持つオブジェクトを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 3, 1>{obj.vx(), obj.vy(), obj.omega()}
template <class T>
inline auto velocity3d(const T& obj) -> Eigen::Matrix<
    std::common_type_t<decltype(obj.vx()), decltype(obj.vy()), decltype(obj.omega())>, 3, 1> {
  return {obj.vx(), obj.vy(), obj.omega()};
}

/// @brief     velocity_tを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    eigen::vector2d{obj.vx, obj.vy, obj.omega}
inline Eigen::Vector3d velocity3d(const model::command::velocity_t& obj) {
  return {obj.vx, obj.vy, obj.omega};
}

/// @brief     メンバ関数ax(), ay()を持つオブジェクトを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 2, 1>{obj.ax(), obj.ay()}
template <class T>
inline auto acceleration(const T& obj)
    -> Eigen::Matrix<std::common_type_t<decltype(obj.ax()), decltype(obj.ay())>, 2, 1> {
  return {obj.ax(), obj.ay()};
}

/// @brief     acceleration_tを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.ax, obj.ay}
inline Eigen::Vector2d acceleration(const model::command::acceleration_t& obj) {
  return {obj.ax, obj.ay};
}

/// @brief     メンバ関数ax(), ay(), alpha()を持つオブジェクトを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<共通の型, 3, 1>{obj.ax(), obj.ay(), obj.alpha()}
template <class T>
inline auto acceleration3d(const T& obj) -> Eigen::Matrix<
    std::common_type_t<decltype(obj.ax()), decltype(obj.ay()), decltype(obj.alpha())>, 3, 1> {
  return {obj.ax(), obj.ay(), obj.alpha()};
}

/// @brief     acceleration_tを3次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.ax, obj.ay, obj.alpha}
inline Eigen::Vector3d acceleration3d(const model::command::acceleration_t& obj) {
  return {obj.ax, obj.ay, obj.alpha};
}

} // namespace math
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_MATH_TO_VECTOR_H
