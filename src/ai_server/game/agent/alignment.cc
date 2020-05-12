#include <cmath>
#include <boost/math/constants/constants.hpp>
#include "alignment.h"

using boost::math::constants::pi;

namespace ai_server {
namespace game {
namespace agent {

alignment::alignment(context& ctx, const std::vector<unsigned int>& ids)
    : base(ctx), ids_(ids) {}

std::vector<std::shared_ptr<action::base>> alignment::execute() {
  std::vector<std::shared_ptr<action::base>> exe;
  const auto our_robots = model::our_robots(world(), team_color());

  // move
  int id_n = 0;
  const double x0 =
      -world().field().x_max() / 2 + 110.0 * (static_cast<double>(ids_.size()) - 1.0);
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
