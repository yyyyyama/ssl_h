#include "halt.h"

namespace ai_server::model::motion {

halt::halt() : base(2) {}

std::tuple<double, double, double> halt::execute() {
  return std::make_tuple<double, double, double>(0.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion