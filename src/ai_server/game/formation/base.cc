#include "base.h"

namespace ai_server {
namespace game {
namespace formation {

base::base(const model::world& world, bool is_yellow, const model::refbox& refcommand)
    : world_(world), is_yellow_(is_yellow), refcommand_(refcommand) {}

} // namespace agent
} // namespace game
} // namespace ai_server