#ifndef AI_SERVER_UTIL_MATH_AFFINE_H
#define AI_SERVER_UTIL_MATH_AFFINE_H

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

/// @brief           ボールの座標，速度，加速度の変換を行う
/// @param matrix    変換行列
/// @param ball      変換する値
/// @return          変換後の値
model::ball transform_all(const Eigen::Affine3d& matrix, const model::ball& ball);

/// @brief           ロボットの座標，速度，加速度の変換を行う
/// @param matrix    変換行列
/// @param robot     変換する値
/// @return          変換後の値
model::robot transform_all(const Eigen::Affine3d& matrix, const model::robot& robot);

/// @brief           Vector2dの座標変換を行う
/// @param matrix    変換行列
/// @param v         変換する値
/// @return          変換後の値
Eigen::Vector2d transform(const Eigen::Affine3d& matrix, const Eigen::Vector2d& v);

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
