#include <cmath>
#include <queue>
#include <vector>

#include "ai_server/util/algorithm.h"
#include "world.h"

namespace ai_server {
namespace model {

world::world() : field_{}, ball_{}, robots_blue_{}, robots_yellow_{} {}

world::world(const world& others) {
  set_field(others.field());
  set_ball(others.ball());
  set_robots_blue(others.robots_blue());
  set_robots_yellow(others.robots_yellow());
}

world& world::operator=(const world& others) {
  set_field(others.field());
  set_ball(others.ball());
  set_robots_blue(others.robots_blue());
  set_robots_yellow(others.robots_yellow());
  return *this;
}

model::field world::field() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return field_;
}

model::ball world::ball() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return ball_;
}

world::robots_list world::robots_blue() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return robots_blue_;
}

world::robots_list world::robots_yellow() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return robots_yellow_;
}

} // namespace model
} // namespace ai_server
