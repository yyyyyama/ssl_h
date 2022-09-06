#ifndef AI_SERVER_MODEL_MOTION_HALT_H
#define AI_SERVER_MODEL_MOTION_HALT_H

#include "base.h"

namespace ai_server::model::motion {

class halt : public base {
public:
  halt();

  std::tuple<double, double, double> execute();
};

} // namespace ai_server::model::motion

#endif