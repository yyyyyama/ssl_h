#ifndef AI_SERVER_MODEL_GEOMETRY_BOX_H
#define AI_SERVER_MODEL_GEOMETRY_BOX_H

#include <type_traits>
#include <utility>
#include <Eigen/Core>

#include "ai_server/util/math/geometry_traits.h"
#include "point.h"

namespace ai_server::model::geometry {

/// @struct   box
/// @brief    矩形領域を表現する型
template <class T>
struct box {
  point<T> min;
  point<T> max;

  /// @brief 任意の型に変換する
  /// @return {min, max}で初期化された値
  template <template <class> class Box, class Point,
            std::enable_if_t<std::is_constructible_v<Box<Point>, Point, Point>,
                             std::nullptr_t> = nullptr>
  constexpr Box<Point> as() const {
    return {min.template as<Point>(), max.template as<Point>()};
  }

  /// @brief 任意の型に変換する
  /// @return {min, max}で初期化された値
  template <template <class, class> class Box, class Point,
            std::enable_if_t<std::is_constructible_v<Box<Point, Point>, Point, Point>,
                             std::nullptr_t> = nullptr>
  constexpr Box<Point, Point> as() const {
    return {min.template as<Point>(), max.template as<Point>()};
  }

  /// @brief std::pair<Point, Point>に変換する
  /// @return {min, max}で初期化された値
  template <class Point = Eigen::Vector2d>
  constexpr auto as_pair() const {
    return as<std::pair, Point>();
  }

  /// @brief boost::geometry::model::box<Point>に変換する
  /// @return {min, max}で初期化された値
  template <class Point = Eigen::Vector2d>
  inline auto as_box() const {
    return as<boost::geometry::model::box, Point>();
  }
};
} // namespace ai_server::model::geometry

#endif
