#ifndef AI_SERVER_UTIL_MATH_GEOMETRY_H
#define AI_SERVER_UTIL_MATH_GEOMETRY_H

#include <cmath>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ai_server {
namespace util {
namespace math {
/// @brief     ある直線の始点と終点が与えられたとき,終点から左右に任意の長さ分ずらした2点を返す
/// @param apex ある直線についての始点
/// @param middle_base ある直線についての終点
/// @param shift 終点からずらしたい長さ
/// @return
/// std::tuple<Eigen::Matrix<T,2,1>,Eigen::Matrix<T,2,1>>{終点から右にずらした点,終点から左にずらした点}
template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
std::tuple<Eigen::Matrix<T, 2, 1>, Eigen::Matrix<T, 2, 1>> calc_isosceles_vertexes(
    const Eigen::Matrix<T, 2, 1>& apex, const Eigen::Matrix<T, 2, 1>& middle_base, T shift) {
  //計算の為に中心にずらした場合の座標
  const Eigen::Matrix<T, 2, 1> after_middle_base{middle_base - apex};

  // x軸から角度
  const auto alpha = std::atan2(after_middle_base.y(), after_middle_base.x());
  //回転行列
  const Eigen::Rotation2D<T> rotate{alpha};

  //移動した先での仮の座標
  const Eigen::Matrix<T, 2, 1> tmp1{after_middle_base.norm(), shift};
  const Eigen::Matrix<T, 2, 1> tmp2{tmp1.x(), -tmp1.y()};

  //回転した後の正しい座標
  return std::make_tuple<Eigen::Matrix<T, 2, 1>, Eigen::Matrix<T, 2, 1>>(
      (rotate * tmp1) + apex, (rotate * tmp2) + apex);
}

/// @brief   ある円と，任意の点から引いたその円への接線があるとき，その接点を求めて返す
/// @param p      円の外側にある任意の点
/// @param center 円の中心
/// @param r      円の半径
/// @return
/// std::tuple<Eigen::Matrix<T,2,1>,Eigen::Matrix<T,2,1>>{点pから見て円の左側の接点,
/// 点pから見て円の右側の接点}
template <class T, std::enable_if_t<std::is_floating_point<T>::value, std::nullptr_t> = nullptr>
inline auto contact_points(const Eigen::Matrix<T, 2, 1>& p,
                           const Eigen::Matrix<T, 2, 1>& center, T r) {
  // 点pからある円の中心へ進むベクトル
  const Eigen::Matrix<T, 2, 1> pc = center - p;

  // ある点と接点の距離
  // 円の中心から接点へ伸びる線と，接点から点pに伸びる線が垂直に交わることを利用
  const T d = std::sqrt(std::pow(pc.norm(), 2) - std::pow(r, 2));

  // 長さを接点までの距離に合わせる
  const Eigen::Matrix<T, 2, 1> pc_dash = d * pc.normalized();

  //回転行列
  const Eigen::Rotation2D<T> rot(std::atan2(r, d));

  return std::make_tuple<Eigen::Matrix<T, 2, 1>, Eigen::Matrix<T, 2, 1>>(
      p + rot * pc_dash, p + rot.inverse() * pc_dash);
}
} // namespace math
} // namespace util
} // namespace ai_server
#endif
