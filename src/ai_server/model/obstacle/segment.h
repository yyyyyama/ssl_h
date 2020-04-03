#ifndef AI_SERVER_MODEL_OBSTACLE_SEGMENT_H
#define AI_SERVER_MODEL_OBSTACLE_SEGMENT_H

#include <boost/geometry/geometries/segment.hpp>
#include <Eigen/Core>
#include "ai_server/util/math/geometry_traits.h"

namespace ai_server::model::obstacle {
struct segment {
  using geometry_type = boost::geometry::model::segment<Eigen::Vector2d>;

  geometry_type geometry;
  double margin;
};
} // namespace ai_server::model::obstacle

#endif
