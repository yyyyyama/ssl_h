#ifndef AI_SERVER_PLANNER_BASE_H
#define AI_SERVER_PLANNER_BASE_H

#include <boost/variant.hpp>
#include "ai_server/model/command.h"

namespace ai_server {
namespace planner {

using position_t     = model::command::position_t;
using velocity_t     = model::command::velocity_t;
using acceleration_t = model::command::acceleration_t;
using target_t       = boost::variant<position_t, velocity_t>;

class base {
protected:
  position_t target_; // 目標(位置or速度)

public:
  base();
  virtual ~base();

  /// @brief  計算後の目標(位置or速度)を返す
  virtual position_t target();
};

} // namespace planner
} // namespace ai_server

#endif // AI_SERVER_PLANNER_BASE_H
