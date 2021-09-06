#ifndef AI_SERVER_MODEL_MOTION_RUN_BACKWARD_H
#define AI_SERVER_MODEL_MOTION_RUN_BACKWARD_H

#include "base.h"

namespace ai_server::model::motion {

class run_backward : public base {
public:
  run_backward();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif