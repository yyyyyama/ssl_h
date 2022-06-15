#include "left_kick.h"

namespace ai_server::model::motion {

left_kick::left_kick() : base(28) {}

std::tuple<double, double, double> left_kick::execute() {
  return std::make_tuple<double, double, double>(-100.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion