#ifndef AI_SERVER_MODEL_MOTION_RUN_FORWARD_H
#define AI_SERVER_MODEL_MOTION_RUN_FORWARD_H

#include "base.h"

namespace ai_server::model::motion {

class run_forward : public base {
public:
  run_forward();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif