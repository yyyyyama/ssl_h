#ifndef AI_SERVER_MODEL_OBSTACLE_BOX_H
#define AI_SERVER_MODEL_OBSTACLE_BOX_H

#include <boost/geometry/geometries/box.hpp>
#include <Eigen/Core>
#include "ai_server/util/math/geometry_traits.h"

namespace ai_server::model::obstacle {
struct box {
  using geometry_type = boost::geometry::model::box<Eigen::Vector2d>;

  geometry_type geometry;
  double margin;
};
} // namespace ai_server::model::obstacle

#endif
