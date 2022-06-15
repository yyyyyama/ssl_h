#include "right_kick.h"

namespace ai_server::model::motion {

right_kick::right_kick() : base(29) {}

std::tuple<double, double, double> right_kick::execute() {
  return std::make_tuple<double, double, double>(-100.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion