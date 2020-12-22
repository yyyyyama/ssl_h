#ifndef AI_SERVER_PLANNER_OBSTACLE_LIST_H
#define AI_SERVER_PLANNER_OBSTACLE_LIST_H

#include <type_traits>
#include <utility>
#include <variant>
#include <vector>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <Eigen/Core>

#include "ai_server/model/obstacle/box.h"
#include "ai_server/model/obstacle/point.h"
#include "ai_server/model/obstacle/segment.h"
#include "detail/geometry_helper.h"
#include "detail/obstacle_tree.h"

namespace ai_server::planner {
class obstacle_list {
public:
  using tree_type =
      detail::tree_type<model::obstacle::point, model::obstacle::segment, model::obstacle::box>;
  using element_type = typename tree_type::value_type;
  // std::variant<ObstacleTypes...>
  using obstacle_type = typename element_type::second_type;

private:
  std::vector<element_type> buffer_;

public:
  /// @brief リストに障害物を追加する
  /// @param 追加する障害物
  void add(obstacle_type o) {
    // 近似Box
    auto env =
        std::visit([](auto&& arg) { return detail::to_envelope(arg.geometry, arg.margin); }, o);
    // リストに追加
    buffer_.emplace_back(std::move(env), std::move(o));
  }

  /// @brief 内部データを取得する
  const std::vector<element_type>& buffer() const {
    return buffer_;
  }

  /// @brief RTreeを構築して返す
  tree_type to_tree() const {
    return {buffer_.begin(), buffer_.end()};
  }
};
} // namespace ai_server::planner

#endif
