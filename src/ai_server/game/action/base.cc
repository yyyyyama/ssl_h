#include "base.h"

namespace ai_server {
namespace game {
namespace action {

base::base(const model::world& world, bool is_yellow, unsigned int id)
    : world_(world), is_yellow_(is_yellow), id_(id) {}

} // namespace action
} // namespace game
} // namespace ai_server
