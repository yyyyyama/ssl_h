#ifndef AI_SERVER_MODEL_MOTION_RUN_RIGHT_H
#define AI_SERVER_MODEL_MOTION_RUN_RIGHT_H

#include "base.h"

namespace ai_server::model::motion {

class run_right : public base {
public:
  run_right();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif