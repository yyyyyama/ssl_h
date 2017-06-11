#include <cmath>
#include <mutex>
#include <queue>
#include <vector>

#include "ai_server/util/algorithm.h"
#include "world.h"

namespace ai_server {
namespace model {

world::world() : field_{}, ball_{}, robots_blue_{}, robots_yellow_{} {}

world::world(model::field&& field, model::ball&& ball, robots_list&& robots_blue,
             robots_list&& robots_yellow)
    : field_(std::move(field)),
      ball_(std::move(ball)),
      robots_blue_(std::move(robots_blue)),
      robots_yellow_(std::move(robots_yellow)) {}

world::world(const world& others) {
  set_field(others.field());
  set_ball(others.ball());
  set_robots_blue(others.robots_blue());
  set_robots_yellow(others.robots_yellow());
 }

world::world(world&& others) {
  std::unique_lock<std::shared_timed_mutex> lock(others.mutex_);
  std::swap(field_, others.field_);
  std::swap(ball_, others.ball_);
  std::swap(robots_blue_, others.robots_blue_);
  std::swap(robots_yellow_, others.robots_yellow_);
}

world& world::operator=(const world& others) {
  std::unique_lock<std::shared_timed_mutex> lock1(mutex_, std::defer_lock);
  std::shared_lock<std::shared_timed_mutex> lock2(others.mutex_, std::defer_lock);
  std::lock(lock1, lock2);
  field_         = others.field_;
  ball_          = others.ball_;
  robots_blue_   = others.robots_blue_;
  robots_yellow_ = others.robots_yellow_;
  return *this;
}

world& world::operator=(world&& others) {
  std::unique_lock<std::shared_timed_mutex> lock1(mutex_, std::defer_lock);
  std::unique_lock<std::shared_timed_mutex> lock2(others.mutex_, std::defer_lock);
  std::lock(lock1, lock2);
  std::swap(field_, others.field_);
  std::swap(ball_, others.ball_);
  std::swap(robots_blue_, others.robots_blue_);
  std::swap(robots_yellow_, others.robots_yellow_);
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
