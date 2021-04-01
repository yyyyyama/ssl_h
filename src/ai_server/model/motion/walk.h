#ifndef AI_SERVER_MODEL_MOTION_WALK_H
#define AI_SERVER_MODEL_MOTION_WALK_H

#include "base.h"

namespace ai_server::model::motion {

class walk : public base {
public:
  walk();

  std::tuple<double, double, double> execute();
};

} // namespace ai_server::model::motion

#endif