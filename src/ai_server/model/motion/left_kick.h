#ifndef AI_SERVER_MODEL_MOTION_LEFT_KICK_H
#define AI_SERVER_MODEL_MOTION_LEFT_KICK_H

#include "base.h"

namespace ai_server::model::motion {

class left_kick : public base {
public:
  left_kick();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif