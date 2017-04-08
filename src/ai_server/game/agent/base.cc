#include "base.h"

namespace ai_server {
namespace game {
namespace agent {

base::base(const model::world& world, bool is_yellow, const std::vector<unsigned int>& ids)
    : world_(world), is_yellow_(is_yellow), ids_(ids) {}

} // namespace agent
} // namespace game
} // namespace ai_server
