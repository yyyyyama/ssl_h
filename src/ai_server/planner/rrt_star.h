#ifndef AI_SERVER_PLANNER_RRT_STAR_H
#define AI_SERVER_PLANNER_RRT_STAR_H

#include <list>
#include <memory>
#include <queue>
#include <vector>

#include "ai_server/model/command.h"
#include "ai_server/model/world.h"
#include "ai_server/planner/base.h"

namespace ai_server {
namespace planner {

class rrt_star : public base {
public:
  rrt_star(const model::world& world);
  //~rrt_star();

  // 節点
  struct node {
    position_t position_;               // 座標
    double cost_;                       // 親ノードまでに必要なコスト
    std::weak_ptr<struct node> parent_; // 親ノードを指すポインタ
  };

  // 障害物
  struct object {
    position_t position_; // 座標
    double r_;            // 物体の半径
  };

  /// @brief  障害物の任意指定
  /// @param  obstacles 障害物(struct object)のvector
  void set_obstacles(const std::vector<object>& obstacles);

  /// @brief  探索関数(木の生成)
  /// @param  start 初期位置
  /// @param  goal  目標位置
  /// @param  max_pos 探索最大座標
  /// @param  min_pos 探索最小座標
  /// @param  search_num  探索を行う回数
  /// @param  max_branch_length 伸ばす枝の最大距離
  /// @param  margin  避けるときのマージン
  void search(const position_t start, const position_t goal,
              const position_t max_pos = position_t{4500.0, 3000.0, 0.0},
              const position_t min_pos = position_t{-4500.0, -3000.0, 0.0},
              const int search_num = 10, const double max_branch_length = 1000.0,
              const double margin = 100.0);

private:
  model::world world_;
  std::vector<std::shared_ptr<struct node>> tree_; // 探索木(nodeの集まり)
  std::shared_ptr<struct node> nearest_node_;      // 次の目標節点
  std::vector<struct object> default_obstacles_; // 固定障害物,セットしなくても避ける
  std::vector<struct object> additional_obstacles_; // 追加障害物,任意にセットして避ける
  std::vector<struct object> all_obstacles_; // 障害物,上記2つの合算
  std::queue<position_t>
      priority_points_; // あるループで生成された最適なルート木，次ループで優先して探索

  /// @brief  障害物が存在するか
  /// @param  start 初期位置
  /// @param  goal  目標位置
  /// @param  margin  避けるときのマージン
  bool obstructed(const position_t start, const position_t goal, const double margin = 100.0);

  /// @brief  ペナルティエリア内に点があるか
  /// @param  point 見たい座標
  /// @param  margin  避けるときのマージン
  bool in_penalty(const position_t point, const double margin = 100.0);

  /// @brief  フィールド外に点があるか
  /// @param  point 見たい座標
  /// @param  margin  避けるときのマージン
  bool out_field(const position_t point, const double margin = 100.0);
};

} // namespace planner
} // namespace ai_server

#endif // AI_SERVER_PLANNER_RRT_STAR_H
