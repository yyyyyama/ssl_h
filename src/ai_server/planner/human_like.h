#ifndef AI_SERVER_PLANNER_HUMAN_LIKE_H
#define AI_SERVER_PLANNER_HUMAN_LIKE_H

#include "base.h"

namespace ai_server::planner {
class human_like : public base {
public:
  /// @brief ロボットが進むことのできる方向の数を設定する
  /// @param count            設定値
  void set_direction_count(int count);

  /// @brief 障害物エリアからの脱出の際にロボットを進ませる最大距離
  /// @param length           設定値
  void set_max_exit_length(double length);

  /// @brief ロボットを最大限進ませるときの距離
  /// @param length           設定値
  void set_max_length(double length);

  /// @brief ロボットを最低限進ませる距離
  /// @param length           設定値
  void set_min_length(double length);

  /// @brief rayを1段階伸ばすときの距離
  /// @param length           設定値
  void set_step_length(double length);

  base::planner_type planner() override;

private:
  // rayを伸ばす方向の数
  int direction_count_ = 16;
  // rayを１段階のばすときの長さ
  double step_length_ = 100.0;
  // rayの最大長さ
  double max_length_ = 4000.0;
  // rayの最小長さ
  double min_length_ = 100.0;
  // 障害物エリアから脱出するときの最大距離
  double max_exit_length_ = 3000.0;
};
} // namespace ai_server::planner

#endif // AI_SERVER_PLANNER_HUMAN_LIKE_H
