#ifndef AI_SERVER_MODEL_MOTION_RUN_LEFT_H
#define AI_SERVER_MODEL_MOTION_RUN_LEFT_H

#include "base.h"

namespace ai_server::model::motion {

class run_left : public base {
public:
  run_left();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif