#ifndef AI_SERVER_PLANNER_BASE_H
#define AI_SERVER_PLANNER_BASE_H

#include <functional>
#include <Eigen/Core>
#include "ai_server/model/field.h"
#include "obstacle_list.h"

namespace ai_server::planner {

class base {
public:
  using result_type  = std::pair<Eigen::Vector2d, double>;
  using planner_type = std::function<result_type(const Eigen::Vector2d&, const Eigen::Vector2d&,
                                                 const obstacle_list&)>;

  base();
  virtual ~base();

  /// @brief 移動可能領域を設定する
  /// @param min_p 移動可能領域の最大座標
  void set_max_pos(const Eigen::Vector2d& max_p);

  /// @brief 移動可能領域設定する
  /// @param max_p 移動可能領域の最小座標
  void set_min_pos(const Eigen::Vector2d& min_p);

  /// @brief フィールドを基にして移動可能領域を設定する
  /// @param max_p フィールド
  /// @param max_p フィールドを広げる量
  void set_area(const model::field& field, double padding);

  /// @brief 経路探索を行う関数オブジェクトを生成する
  virtual planner_type planner() = 0;

protected:
  // 移動可能領域
  Eigen::Vector2d max_pos_;
  Eigen::Vector2d min_pos_;
};
} // namespace ai_server::planner

#endif // AI_SERVER_PLANNER_BASE_H
