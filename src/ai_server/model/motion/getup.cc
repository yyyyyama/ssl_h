#include "getup.h"

namespace ai_server::model::motion {

getup::getup() : base(54) {}

std::tuple<double, double, double> getup::execute() {
  return std::make_tuple<double, double, double>(0.0, 0.0, 0.0);
}

} // namespace ai_server::model::motion