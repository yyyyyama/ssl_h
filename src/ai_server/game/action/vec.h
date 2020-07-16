#ifndef AI_SERVER_GAME_ACTION_VEC_H
#define AI_SERVER_GAME_ACTION_VEC_H

#include <Eigen/Core>
#include "base.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace action {

class vec : public base {
public:
  vec(context& ctx, unsigned int id);

  // 目的速度を指定する
  void move_at(double vx, double vy, double omega = 0.0);
  void move_at(const Eigen::Vector2d& v, double omega = 0.0);
  void move_at(const Eigen::Vector3d& v);

  model::command execute() override;

  bool finished() const override;

  // 目的速度を取得する
  Eigen::Vector3d velocity() const;

private:
  // 目標速度
  Eigen::Vector3d velocity_;
};
} // namespace action
} // namespace game
} // namespace ai_server

#endif
