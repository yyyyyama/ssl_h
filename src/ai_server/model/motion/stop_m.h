#ifndef AI_SERVER_MODEL_MOTION_STOP_M_H
#define AI_SERVER_MODEL_MOTION_STOP_M_H

#include "base.h"

namespace ai_server::model::motion {

class stop_m : public base {
public:
  stop_m();

  std::tuple<double, double, double> execute();
};

} // namespace ai_server::model::motion

#endif