#ifndef AI_SERVER_MODEL_MOTION_WALK_LEFT_H
#define AI_SERVER_MODEL_MOTION_WALK_LEFT_H

#include "base.h"

namespace ai_server::model::motion {

class walk_left : public base {
public:
  walk_left();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif