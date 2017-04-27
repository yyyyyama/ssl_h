#include <memory>
#include <vector>
#include "ai_server/game/agent/base.h"
#include "ai_server/model/world.h"

namespace ai_server {
namespace game {
namespace agent {

class stopgame : public base {
private:
  const std::vector<unsigned int>& ids_;
  unsigned int nearest_robot = 33;
  std::vector<std::shared_ptr<action::base>> baseaction;

public:
  stopgame(const model::world& world, bool is_yellow, const std::vector<unsigned int>& ids);
  std::vector<std::shared_ptr<action::base>> execute() override;
};

} // agent
} // game
} // ai_server
