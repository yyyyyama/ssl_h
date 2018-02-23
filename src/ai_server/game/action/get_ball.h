#ifndef AI_SERVER_GAME_ACTION_GET_BALL_H
#define AI_SERVER_GAME_ACTION_GET_BALL_H

#include <Eigen/Dense>

#include "ai_server/model/command.h"
#include "base.h"

namespace ai_server {
namespace game {
namespace action {

class get_ball : public base {
public:
  get_ball(const model::world& world, bool is_yellow, unsigned int id);
  get_ball(const model::world& world, bool is_yellow, unsigned int id, Eigen::Vector2d target);
  void set_target(double x, double y);
  void set_pow(double pow);
  void set_chip(bool chip);
  Eigen::Vector2d target() const;
  double pow() const;
  bool chip() const;
  model::command execute() override;
  bool finished() const override;

private:
  Eigen::Vector2d target_; // 蹴りたい目標
  //蹴る強さ
  double pow_;
  //終了フラグ
  bool flag_;
  //チップキックにするかどうか
  bool chip_;
};

} // namespace action
} // namespace game
} // namespace ai_server

#endif // AI_SERVER_GAME_ACTION_GET_BALL_H
