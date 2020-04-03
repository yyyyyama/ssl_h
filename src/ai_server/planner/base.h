#ifndef AI_SERVER_PLANNER_BASE_H
#define AI_SERVER_PLANNER_BASE_H

#include <functional>
#include <Eigen/Core>
#include "ai_server/model/command.h"
#include "obstacle_list.h"

namespace ai_server::planner {

using position_t     = model::command::position_t;
using velocity_t     = model::command::velocity_t;
using acceleration_t = model::command::acceleration_t;

class base {
public:
  using result_type = std::pair<position_t, double>;
  using planner_type =
      std::function<result_type(const position_t&, const position_t&, const obstacle_list&)>;

  base();
  virtual ~base();

  /// @brief 移動可能領域を設定する
  /// @param min_p 移動可能領域の最大座標
  void set_max_pos(const Eigen::Vector2d& max_p);

  /// @brief 移動可能領域設定する
  /// @param max_p 移動可能領域の最小座標
  void set_min_pos(const Eigen::Vector2d& min_p);

  /// @brief マージンの設定
  /// @param m マージン
  void set_margin(double m);

  /// @brief 経路探索を行う関数オブジェクトを生成する
  virtual planner_type planner() = 0;

protected:
  // 移動可能領域
  Eigen::Vector2d max_pos_;
  Eigen::Vector2d min_pos_;

  // 障害物に対するマージン
  double margin_;
};
} // namespace ai_server::planner

#endif // AI_SERVER_PLANNER_BASE_H
