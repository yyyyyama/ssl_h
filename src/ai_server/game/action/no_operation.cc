#include "no_operation.h"

namespace ai_server {
namespace game {
namespace action {

model::command no_operation::execute() {
  return {};
}

bool no_operation::finished() const {
  return true;
}
} // namespace action
} // namespace game
} // namespace ai_server
