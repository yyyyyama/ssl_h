#include "stop.h"

namespace ai_server::model::motion {

stop::stop() : base(0) {}

std::tuple<double, double, double> stop::execute() {
  return std::make_tuple<double, double, double>(0.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion