#ifndef AI_SERVER_GAME_AGENT_PERFORMANCE_H
#define AI_SERVER_GAME_AGENT_PERFORMANCE_H

#include <map>
#include <vector>
#include <Eigen/Core>

#include "ai_server/model/world.h"
#include "base.h"

namespace ai_server::game::agent {

class performance : public base {
public:
  /// @brief コンストラクタ
  /// @param ctx ボールやロボットの情報
  /// @param ids 使うロボットのid
  performance(context& ctx, const std::vector<unsigned int>& ids);

  /// @brief セッター
  /// @param 最大半径
  /// @param 最小半径
  /// @param 基準点
  void set_area(double max_rad, double min_rad, const Eigen::Vector2d& base_pos);

  /// @brief 実行
  /// @return ロボットに送信するコマンド
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  const std::vector<unsigned int> ids_;
  // ロボットが消えた時点
  std::map<unsigned int, std::chrono::steady_clock::time_point> lost_point_;
  const std::chrono::steady_clock::time_point start_point_;
  double max_rad_;
  double min_rad_;
  Eigen::Vector2d base_pos_;
};
} // namespace ai_server::game::agent

#endif
