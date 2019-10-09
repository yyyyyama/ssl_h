#ifndef AI_SERVER_PLANNER_BASE_H
#define AI_SERVER_PLANNER_BASE_H

#include "ai_server/model/command.h"

namespace ai_server {
namespace planner {

using position_t     = model::command::position_t;
using velocity_t     = model::command::velocity_t;
using acceleration_t = model::command::acceleration_t;

class base {
protected:
  position_t target_; // 目標(位置or速度)

  double trajectory_length_; // 経路長

public:
  base();
  virtual ~base();

  /// @brief  計算後の目標(位置or速度)を返す
  virtual position_t target();

  /// @brief  経路長を返す
  virtual double trajectory_length();
};

} // namespace planner
} // namespace ai_server

#endif // AI_SERVER_PLANNER_BASE_H
