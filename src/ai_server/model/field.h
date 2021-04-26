#ifndef AI_SERVER_MODEL_FIELD_H
#define AI_SERVER_MODEL_FIELD_H

#include "geometry/box.h"
#include "geometry/point.h"

namespace ai_server {
namespace model {
class field {
private:
  int length_;
  int width_;
  int center_radius_;
  int goal_width_;
  int penalty_length_;
  int penalty_width_;

public:
  using box   = geometry::box<double>;
  using point = geometry::point<double>;

  field();

  // :::::::::::::::::::::: 位置関係対応図 ::::::::::::::::::::::
  //
  //                          相手側
  //
  //            (x_max, y_max)              (x_max, y_min)
  //             *-----------------------------* - - - - - - - -
  //             |                             |             ^
  //             |                             |             ^
  //             |                             |             '
  //             |           [front]           |             '
  //             |                             |          length
  //             |                             |             '
  //             | [left]   [center]   [right] |             '
  //  (0, y_max) *------------- O -------------* (0, y_min)  '
  //             |                             |             '
  //             |                             |             '
  //             |           [back]            |             '
  //             |                             |             '
  //             |                             |             '
  //             |                             |             '
  //             | < - - - width - - - - - - > |             v
  //             *-----------------------------* - - - - - - - -
  //            (x_min, y_max)              (x_min, y_min)
  //
  //                          自陣側
  //

  /// @brief フィールドの前後長さ
  int length() const;
  /// @brief フィールドの左右幅
  int width() const;
  /// @brief センターサークルの半径
  int center_radius() const;
  /// @brief ゴールの左右幅
  int goal_width() const;
  /// @brief ペナルティエリアの前後長さ
  int penalty_length() const;
  /// @brief ペナルティエリアの左右幅
  int penalty_width() const;

  void set_length(int length);
  void set_width(int width);
  void set_center_radius(int center_radius);
  void set_goal_width(int goal_width);
  void set_penalty_length(int penalty_length);
  void set_penalty_width(int penalty_width);

  /// @brief フィールドのx座標最大値
  double x_max() const;
  /// @brief フィールドのx座標最小値
  double x_min() const;
  /// @brief フィールドのy座標最大値
  double y_max() const;
  /// @brief フィールドのy座標最小値
  double y_min() const;

  /// @brief 後方ペナルティエリアのx座標最大値
  double back_penalty_x() const;
  /// @brief 前方ペナルティエリアのx座標最小値
  double front_penalty_x() const;
  /// @brief ペナルティエリアのy座標最大値
  double penalty_y_max() const;
  /// @brief ペナルティエリアのy座標最小値
  double penalty_y_min() const;

  /// @brief ゴールのy座標最大値
  double goal_y_max() const;
  /// @brief ゴールのy座標最小値
  double goal_y_min() const;

  /// @brief 後方ゴール中心の座標
  point back_goal_center() const;
  /// @brief 後方ゴール左端の座標
  point back_goal_left() const;
  /// @brief 後方ゴール右端の座標
  point back_goal_right() const;

  /// @brief 前方ゴール中心の座標
  point front_goal_center() const;
  /// @brief 前方ゴール左端の座標
  point front_goal_left() const;
  /// @brief 前方ゴール右端の座標
  point front_goal_right() const;

  /// @brief 後方左側コーナーの座標
  point back_left_corner() const;
  /// @brief 後方右側コーナーの座標
  point back_right_corner() const;

  /// @brief 前方左側コーナーの座標
  point front_left_corner() const;
  /// @brief 前方右側コーナーの座標
  point front_right_corner() const;

  /// @brief 後方ペナルティマークの座標
  point back_penalty_mark() const;
  /// @brief 前方ペナルティマークの座標
  point front_penalty_mark() const;

  /// @brief 後方ハーフエリア
  box back_half_area() const;
  /// @brief 前方ハーフエリア
  box front_half_area() const;

  /// @brief 後方ペナルティエリア
  box back_penalty_area() const;
  /// @brief 前方ペナルティエリア
  box front_penalty_area() const;

  /// @brief フィールド全体
  box game_area() const;
};
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_FIELD_H
