#ifndef AI_SERVER_MODEL_MOTION_TURN_LEFT_H
#define AI_SERVER_MODEL_MOTION_TURN_LEFT_H

#include "base.h"

namespace ai_server::model::motion {

class turn_left : public base {
public:
  turn_left();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif