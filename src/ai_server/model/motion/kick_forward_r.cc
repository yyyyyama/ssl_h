#include "kick_forward_r.h"

namespace ai_server::model::motion {

kick_forward_r::kick_forward_r() : base(27) {}

std::tuple<double, double, double> kick_forward_r::execute() {
  return std::make_tuple<double, double, double>(100.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion