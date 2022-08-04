#ifndef AI_SERVER_MODEL_MOTION_GETUP_H
#define AI_SERVER_MODEL_MOTION_GETUP_H

#include "base.h"

namespace ai_server::model::motion {

class getup : public base {
public:
  getup();

  std::tuple<double, double, double> execute();
};

} // namespace ai_server::model::motion

#endif