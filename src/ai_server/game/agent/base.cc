#include "base.h"

namespace ai_server {
namespace game {
namespace agent {

base::base(const model::world& world, bool is_yellow) : world_(world), is_yellow_(is_yellow) {}

} // namespace agent
} // namespace game
} // namespace ai_server
