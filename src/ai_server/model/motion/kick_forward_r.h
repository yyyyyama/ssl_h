#ifndef AI_SERVER_MODEL_MOTION_KICK_FORARD_R_H
#define AI_SERVER_MODEL_MOTION_KICK_FORARD_R_H

#include "base.h"

namespace ai_server::model::motion {

class kick_forward_r : public base {
public:
  kick_forward_r();

  std::tuple<double, double, double> execute() override;
};

} // namespace ai_server::model::motion

#endif