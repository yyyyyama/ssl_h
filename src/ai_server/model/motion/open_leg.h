#ifndef AI_SERVER_MODEL_MOTION_OPEN_LEG_H
#define AI_SERVER_MODEL_MOTION_OPEN_LEG_H

#include "base.h"

namespace ai_server::model::motion {

class open_leg : public base {
public:
  open_leg();

  std::tuple<double, double, double> execute();
};

} // namespace ai_server::model::motion

#endif
