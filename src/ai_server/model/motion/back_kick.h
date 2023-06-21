#ifndef AI_SERVER_MODEL_BACK_KICK_H
#define AI_SERVER_MODEL_BACK_KICK_H

#include "base.h"

namespace ai_server::model::motion {

class back_kick : public base {
public:
  back_kick();

  std::tuple<double, double, double> execute();
};

} // namespace ai_server::model::motion

#endif