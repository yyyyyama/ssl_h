#include "turn_left.h"

namespace ai_server::model::motion {

turn_left::turn_left() : base(20) {}

std::tuple<double, double, double> turn_left::execute() {
  return std::make_tuple<double, double, double>(0.0, 0.0, 1.5);
}

} // namespace ai_server::model::motion