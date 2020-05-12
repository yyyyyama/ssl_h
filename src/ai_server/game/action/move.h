#ifndef AI_SERVER_GAME_ACTION_MOVE_H
#define AI_SERVER_GAME_ACTION_MOVE_H

#include <Eigen/Core>
#include "base.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace action {

class move : public base {
public:
  // マージンの情報
  struct margin_t {
    // 位置マージン
    double position;
    // 角度マージン[rad]
    double theta;
  };

  move(context& ctx, unsigned int id);

  // 目的地を指定する
  void move_to(double x, double y, double theta = 0.0);
  void move_to(const Eigen::Vector2d& pos, double theta = 0.0);
  void move_to(const Eigen::Vector3d& target);

  // 許容誤差を指定する
  void set_margin(const margin_t& margin);

  model::command execute() override;

  bool finished() const override;

  // 許容誤差を取得する
  margin_t margin() const;

  // 目的地を取得する
  Eigen::Vector3d target() const;

private:
  // 目的地
  Eigen::Vector3d target_;
  // 許容誤差
  margin_t margin_;
  // 終了フラグ
  bool finished_;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif
