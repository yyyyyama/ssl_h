#ifndef AI_SERVER_MODEL_MOTION_WALK_RIGHT_H
#define AI_SERVER_MODEL_MOTION_WALK_RIGHT_H

#include "base.h"

namespace ai_server::model::motion {

class walk_right : public base {
public:
  walk_right();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif