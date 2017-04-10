#include "no_operation.h"

namespace ai_server {
namespace game {
namespace action {

model::command no_operation::execute() {
  return model::command{id_};
}

bool no_operation::finished() const {
  return true;
}
}
}
}
