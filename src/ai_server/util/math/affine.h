#ifndef AI_SERVER_UTIL_MATH_AFFINE_H
#define AI_SERVER_UTIL_MATH_AFFINE_H

#include <tuple>
#include <utility>
#include <Eigen/Geometry>

#include "ai_server/model/ball.h"
#include "ai_server/model/robot.h"

namespace ai_server {
namespace util {
namespace math {

/// @brief           ボールの座標変換を行う
/// @param matrix    変換行列
/// @param ball      変換する値
/// @return          変換後の値
model::ball transform(const Eigen::Affine3d& matrix, const model::ball& ball);

/// @brief           ロボットの座標変換を行う
/// @param matrix    変換行列
/// @param robot     変換する値
/// @return          変換後の値
model::robot transform(const Eigen::Affine3d& matrix, const model::robot& robot);

namespace detail {
template <class Vector, std::size_t... Idx>
auto vector_to_tuple_impl(const Vector& v, std::index_sequence<Idx...>) {
  return std::make_tuple(v(Idx)...);
}
template <std::size_t TupleLength, class Derived>
auto vector_to_tuple(const Eigen::MatrixBase<Derived>& v) {
  static_assert(Derived::RowsAtCompileTime != Eigen::Dynamic);
  static_assert(Derived::ColsAtCompileTime == 1);
  static_assert(TupleLength <= Derived::RowsAtCompileTime);
  return vector_to_tuple_impl(v.derived(), std::make_index_sequence<TupleLength>{});
}
} // namespace detail

/// @brief           std::tuple<...>の座標変換を行う
/// @param matrix    変換行列
/// @param robot     変換する値
/// @return          変換後の値
/// tuple に対して ball や robot と同等の座標変換を行う
/// ただし、3つ目の要素の値に対し wrap_to_2pi は適用されない
template <class T, class... Ts, int N>
auto transform(const Eigen::Transform<T, N, Eigen::Affine>& matrix,
               const std::tuple<Ts...>& tuple)
    -> std::enable_if_t<std::conjunction_v<std::is_same<T, Ts>...> &&
                            (static_cast<int>(sizeof...(Ts)) <= N),
                        std::tuple<Ts...>> {
  constexpr auto tuple_length = sizeof...(Ts);
  Eigen::Matrix<T, N, 1> v    = Eigen::Matrix<T, N, 1>::Zero();
  {
    auto sv = v.template head<static_cast<int>(tuple_length)>();
    auto f  = [&sv](auto&& x, auto&&... xs) {
      ((sv << std::forward<decltype(x)>(x)), ..., std::forward<decltype(xs)>(xs)).finished();
    };
    std::apply(f, tuple);
  }
  return detail::vector_to_tuple<tuple_length>(matrix * v);
}

/// @brief           変換行列を作る
/// @param x         x軸方向に平行移動する量
/// @param y         y軸方向に平行移動する量
/// @param theta     z軸を中心に回転する量
/// @return          変換行列
template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
auto make_transformation_matrix(T x, T y, T theta) -> Eigen::Transform<T, 3, Eigen::Affine> {
  return Eigen::Translation<T, 3>{x, y, -theta} *
         Eigen::AngleAxis<T>{theta, Eigen::Matrix<T, 3, 1>::UnitZ()};
}

} // namespace math
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_MATH_AFFINE_H
