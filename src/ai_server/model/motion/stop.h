#ifndef AI_SERVER_MODEL_MOTION_STOP_H
#define AI_SERVER_MODEL_MOTION_STOP_H

#include "base.h"

namespace ai_server::model::motion {

class stop : public base {
public:
  stop();

  std::tuple<double, double, double> execute();
};

} // namespace ai_server::model::motion

#endif