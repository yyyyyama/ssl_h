#include <cmath>
#include <queue>
#include <vector>

#include "ai_server/util/algorithm.h"
#include "world.h"

namespace ai_server {
namespace model {

model::field world::field() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return field_;
}

model::ball world::ball() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ball_;
}

world::robots_list world::robots_blue() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return robots_blue_;
}

world::robots_list world::robots_yellow() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return robots_yellow_;
}

} // namespace model
} // namespace ai_server
