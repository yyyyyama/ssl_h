#include "kick_forward_l.h"

namespace ai_server::model::motion {

kick_forward_l::kick_forward_l() : base(26) {}

std::tuple<double, double, double> kick_forward_l::execute() {
  return std::make_tuple<double, double, double>(100.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion