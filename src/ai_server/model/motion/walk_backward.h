#ifndef AI_SERVER_MODEL_MOTION_WALK_BACKWARD_H
#define AI_SERVER_MODEL_MOTION_WALK_BACKWARD_H

#include "base.h"

namespace ai_server::model::motion {

class walk_backward : public base {
public:
  walk_backward();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif