#ifndef AI_SERVER_MODEL_OBSTACLE_FIELD_H
#define AI_SERVER_MODEL_OBSTACLE_FIELD_H

#include "ai_server/model/field.h"
#include "box.h"

namespace ai_server::model::obstacle {

/// @brief 自陣側ペナルティエリアを障害物として取得する
/// @param field フィールドの情報
/// @param margin ペナリティエリアのマージン
/// @return
static inline obstacle::box our_penalty_area(const model::field& field, double margin) {
  // 移動可能領域からはみ出させるために伸ばす量 (大きめの値を指定)
  constexpr double over_length = 100000.0;

  const double field_bottom = field.x_min();
  const double half_width   = field.penalty_width() * 0.5;
  const double length       = field.penalty_length();

  return {{{field_bottom - over_length, -half_width}, {field_bottom + length, half_width}},
          margin};
}

/// @brief 敵陣側ペナルティエリアを障害物として取得する
/// @param field フィールドの情報
/// @param margin ペナリティエリアのマージン
/// @return
static inline obstacle::box enemy_penalty_area(const model::field& field, double margin) {
  // 移動可能領域からはみ出させるために伸ばす量 (大きめの値を指定)
  constexpr double over_length = 100000.0;

  const double field_top  = field.x_max();
  const double half_width = field.penalty_width() * 0.5;
  const double length     = field.penalty_length();

  return {{{field_top - length, -half_width}, {field_top + over_length, half_width}}, margin};
}
} // namespace ai_server::model::obstacle

#endif
