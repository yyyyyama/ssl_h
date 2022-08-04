#include "turn_right.h"

namespace ai_server::model::motion {

turn_right::turn_right() : base(21) {}

std::tuple<double, double, double> turn_right::execute() {
  return std::make_tuple<double, double, double>(0.0, 0.0, -1.5);
}

} // namespace ai_server::model::motion