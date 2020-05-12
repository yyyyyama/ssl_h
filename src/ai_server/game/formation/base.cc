#include "base.h"

namespace ai_server {
namespace game {
namespace formation {

base::base(context& ctx, const model::refbox& refcommand)
    : ctx_(ctx), refcommand_(refcommand) {}

} // namespace formation
} // namespace game
} // namespace ai_server