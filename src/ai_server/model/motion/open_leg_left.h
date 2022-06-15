#ifndef AI_SERVER_MODEL_MOTION_OPEN_LEG_LEFT_H
#define AI_SERVER_MODEL_MOTION_OPEN_LEG_LEFT_H

#include "base.h"

namespace ai_server::model::motion {

class open_leg_left : public base {
public:
  open_leg_left();

  std::tuple<double, double, double> execute();
};

} // namespace ai_server::model::motion

#endif