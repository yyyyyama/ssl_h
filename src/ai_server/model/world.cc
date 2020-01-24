#include "world.h"

namespace ai_server {
namespace model {

world::world(model::field&& field, model::ball&& ball, robots_list&& robots_blue,
             robots_list&& robots_yellow)
    : field_(std::move(field)),
      ball_(std::move(ball)),
      robots_blue_(std::move(robots_blue)),
      robots_yellow_(std::move(robots_yellow)) {}

model::field world::field() const {
  return field_;
}

model::ball world::ball() const {
  return ball_;
}

world::robots_list world::robots_blue() const {
  return robots_blue_;
}

world::robots_list world::robots_yellow() const {
  return robots_yellow_;
}

} // namespace model
} // namespace ai_server
