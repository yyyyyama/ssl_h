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

  return {{{field.x_min() - over_length, field.penalty_y_min()},
           {field.back_penalty_x(), field.penalty_y_max()}},
          margin};
}

/// @brief 敵陣側ペナルティエリアを障害物として取得する
/// @param field フィールドの情報
/// @param margin ペナリティエリアのマージン
/// @return
static inline obstacle::box enemy_penalty_area(const model::field& field, double margin) {
  // 移動可能領域からはみ出させるために伸ばす量 (大きめの値を指定)
  constexpr double over_length = 100000.0;

  return {{{field.front_penalty_x(), field.penalty_y_min()},
           {field.x_max() + over_length, field.penalty_y_max()}},
          margin};
}
} // namespace ai_server::model::obstacle

#endif
