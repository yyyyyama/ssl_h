#ifndef AI_SERVER_MODEL_OBSTACLE_POINT_H
#define AI_SERVER_MODEL_OBSTACLE_POINT_H

#include <Eigen/Core>

namespace ai_server::model::obstacle {
struct point {
  using geometry_type = Eigen::Vector2d;

  geometry_type geometry;
  double margin;
};
} // namespace ai_server::model::obstacle

#endif
