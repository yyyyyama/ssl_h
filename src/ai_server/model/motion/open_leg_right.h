#ifndef AI_SERVER_MODEL_MOTION_OPEN_LEG_RIGHT_H
#define AI_SERVER_MODEL_MOTION_OPEN_LEG_RIGHT_H

#include "base.h"

namespace ai_server::model::motion {

class open_leg_right : public base {
public:
  open_leg_right();

  std::tuple<double, double, double> execute();
};

} // namespace ai_server::model::motion

#endif