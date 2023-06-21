#include "back_kick.h"

namespace ai_server::model::motion {

back_kick::back_kick() : base(30) {}

std::tuple<double, double, double> back_kick::execute() {
  return std::make_tuple<double, double, double>(0.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion