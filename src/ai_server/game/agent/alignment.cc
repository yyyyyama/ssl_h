#include <cmath>
#include <boost/math/constants/constants.hpp>
#include "alignment.h"

using boost::math::constants::pi;

namespace ai_server {
namespace game {
namespace agent {

alignment::alignment(const model::world& world, bool is_yellow,
                     const std::vector<unsigned int>& ids)
    : base(world, is_yellow), ids_(ids) {}

std::vector<std::shared_ptr<action::base>> alignment::execute() {
  std::vector<std::shared_ptr<action::base>> exe;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();

  // move
  int id_n = 0;
  const double x0 =
      -world_.field().x_max() / 2 + 110.0 * (static_cast<double>(ids_.size()) - 1.0);
  for (auto id : ids_) {
    if (our_robots.count(id)) {
      auto move          = make_action<action::move>(id);
      const double y     = 0;
      const double x     = x0 - 220.0 * id_n;
      const double theta = pi<double>() * 1.5;
      move->move_to(x, y, theta);
      exe.push_back(move);
    }
    id_n++;
  }
  return exe;
}
} // namespace agent
} // namespace game
} // namespace ai_server
