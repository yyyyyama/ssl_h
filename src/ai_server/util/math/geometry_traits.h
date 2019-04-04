#ifndef AI_SERVER_UTIL_MATH_GEOMETRY_TRAITS_H
#define AI_SERVER_UTIL_MATH_GEOMETRY_TRAITS_H

#include <boost/geometry/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Eigen::Matrix<T, DimensionCount, 1>を、boost::geometory::model::pointとして使えるようにする
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

template <class T, int DimensionCount, int CoordinateAxis>
struct access<Eigen::Matrix<T, DimensionCount, 1>, CoordinateAxis> {
  static T get(const Eigen::Matrix<T, DimensionCount, 1>& p) {
    return p(CoordinateAxis);
  }

  static void set(Eigen::Matrix<T, DimensionCount, 1>& p, T value) {
    p(CoordinateAxis) = value;
  }
};
} // namespace boost::geometry::traits

#endif
