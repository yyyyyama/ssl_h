#ifndef AI_SERVER_PLANNER_RRT_STAR_H
#define AI_SERVER_PLANNER_RRT_STAR_H

#include "base.h"

namespace ai_server::planner {
class rrt_star : public base {
public:
  rrt_star();

  /// @brief ノードを作る回数を設定する
  /// @param count 設定値．
  void set_node_count(int count);

  /// @brief 伸ばす枝の最大距離を設定する
  /// @param length 設定値．
  void set_max_branch_length(double length);

  base::planner_type planner() override;

private:
  // 探索を行う回数
  int node_count_;

  // 伸ばす枝の最大距離
  double max_branch_length_;
};
} // namespace ai_server::planner

#endif // AI_SERVER_PLANNER_RRT_STAR_H
