#ifndef AI_SERVER_PLANNER_DETAIL_OBSTACLE_TREE_H
#define AI_SERVER_PLANNER_DETAIL_OBSTACLE_TREE_H

#include <utility>
#include <variant>
#include <boost/geometry/index/rtree.hpp>

#include "geometry_helper.h"

namespace ai_server::planner::detail {

// 障害物RTree
// 障害物を包括するboxと，障害物のペアを保持
template <class... ObstacleTypes>
using tree_type =
    boost::geometry::index::rtree<std::pair<envelope_type, std::variant<ObstacleTypes...>>,
                                  boost::geometry::index::rstar<20>>;
} // namespace ai_server::planner::detail

#endif
