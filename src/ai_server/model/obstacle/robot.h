#ifndef AI_SERVER_MODEL_OBSTACLE_ROBOT_H
#define AI_SERVER_MODEL_OBSTACLE_ROBOT_H

#include <chrono>
#include <type_traits>

#include "ai_server/model/world.h"
#include "ai_server/planner/obstacle_list.h"
#include "ai_server/util/math/to_vector.h"
#include "point.h"

namespace ai_server::model::obstacle {

/// @brief ロボットを障害物リストに追加する
/// @param   obstacles 追加先
/// @param   robots    障害物として登録するロボット
/// @param   radius    障害物の半径
static inline void add_robots(planner::obstacle_list& obstacles,
                              const world::robots_list& robots, double radius) {
  for (const auto& r : robots) {
    obstacles.add(obstacle::point{util::math::position(r.second), radius});
  }
}

/// @brief 条件を満たすロボットを障害物リストに追加する
/// @param   obstacles 追加先
/// @param   robots    障害物として登録するロボット
/// @param   radius    障害物の半径
/// @param   pred      条件を満たすとき true を返す関数
template <
    class Predicate,
    std::enable_if_t<std::is_invocable_r_v<bool, Predicate, unsigned int, const model::robot&>,
                     std::nullptr_t> = nullptr>
static inline void add_robots_if(planner::obstacle_list& obstacles,
                                 const world::robots_list& robots, double radius,
                                 Predicate&& pred) {
  for (const auto& [id, r] : robots) {
    if (pred(id, r)) obstacles.add(obstacle::point{util::math::position(r), radius});
  }
}

/// @brief ロボットの軌跡を障害物リストに追加する
/// @param   obstacles 追加先
/// @param   robots    障害物として登録するロボット
/// @param   radius    障害物の半径
/// @param   from      軌跡が始まるタイミング
/// @param   to        軌跡が終わるタイミング
static inline void add_robot_paths(planner::obstacle_list& obstacles,
                                   const world::robots_list& robots, double radius,
                                   std::chrono::system_clock::duration from,
                                   std::chrono::system_clock::duration to) {
  for (const auto& r : robots) {
    const auto p = util::math::position(r.second);
    const auto v = util::math::velocity(r.second);

    obstacles.add(obstacle::segment{{p + std::chrono::duration<double>(from).count() * v,
                                     p + std::chrono::duration<double>(to).count() * v},
                                    radius});
  }
}

/// @brief 条件を満たすロボットの軌跡を障害物リストに追加する
/// @param   obstacles 追加先
/// @param   robots    障害物として登録するロボット
/// @param   radius    障害物の半径
/// @param   from      軌跡が始まるタイミング
/// @param   to        軌跡が終わるタイミング
/// @param   pred      条件を満たすとき true を返す関数
template <
    class Predicate,
    std::enable_if_t<std::is_invocable_r_v<bool, Predicate, unsigned int, const model::robot&>,
                     std::nullptr_t> = nullptr>
static inline void add_robot_paths_if(planner::obstacle_list& obstacles,
                                      const world::robots_list& robots, double radius,
                                      std::chrono::system_clock::duration from,
                                      std::chrono::system_clock::duration to,
                                      Predicate&& pred) {
  for (const auto& [id, r] : robots) {
    if (pred(id, r)) {
      const auto p = util::math::position(r);
      const auto v = util::math::velocity(r);

      obstacles.add(obstacle::segment{{p + std::chrono::duration<double>(from).count() * v,
                                       p + std::chrono::duration<double>(to).count() * v},
                                      radius});
    }
  }
}

} // namespace ai_server::model::obstacle

#endif
