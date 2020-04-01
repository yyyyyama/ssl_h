#ifndef AI_SERVER_UTIL_MATH_GEOMETRY_TRAITS_H
#define AI_SERVER_UTIL_MATH_GEOMETRY_TRAITS_H

#include <cstddef>
// boost/geometry/algorithms/* が使えるようにする
#include <boost/geometry/strategies/strategies.hpp>
#include <Eigen/Core>

// Eigen::Matrix<T, DimensionCount, 1> を
// boost::geometory::model::point<T , DimensionCount, cs::cartesian> と同等に扱えるようにする

// 参考: boost/geometry/geometries/register/point.hpp

namespace boost::geometry::traits {

template <class T, int DimensionCount>
struct tag<Eigen::Matrix<T, DimensionCount, 1>> {
  using type = point_tag;
};

template <class T, int DimensionCount>
struct coordinate_type<Eigen::Matrix<T, DimensionCount, 1>> {
  using type = T;
};

template <class T, int DimensionCount>
struct coordinate_system<Eigen::Matrix<T, DimensionCount, 1>> {
  using type = cs::cartesian;
};

template <class T, int DimensionCount>
struct dimension<Eigen::Matrix<T, DimensionCount, 1>> : boost::mpl::int_<DimensionCount> {};

template <class T, int DimensionCount, std::size_t CoordinateAxis>
struct access<Eigen::Matrix<T, DimensionCount, 1>, CoordinateAxis> {
  static_assert(CoordinateAxis < DimensionCount);

  static inline T get(const Eigen::Matrix<T, DimensionCount, 1>& p) {
    return p.coeff(CoordinateAxis);
  }

  static inline void set(Eigen::Matrix<T, DimensionCount, 1>& p, T value) {
    p.coeffRef(CoordinateAxis) = value;
  }
};
} // namespace boost::geometry::traits

#endif
