#ifndef AI_SERVER_GAME_AGENT_SETPLAY_H
#define AI_SERVER_GAME_AGENT_SETPLAY_H
#include <memory>
#include <vector>
#include <Eigen/Core>
#include "base.h"
#include "ai_server/model/world.h"
#include "ai_server/game/action/kick_action.h"

namespace ai_server {
namespace game {
namespace agent {

class setplay : public base {
public:
  enum class state { setup, pass, shoot, finished };
  state state_ = state::setup;
  enum class pos { near, mid, far };
  pos pos_;
  setplay(const model::world& world, bool is_yellow, unsigned int kicker_id,
          const std::vector<unsigned int>& receiver_id);
  void set_extra_robots(const std::vector<unsigned int>& ids);
  std::vector<std::shared_ptr<action::base>> execute() override;
  bool finished();

private:
  unsigned int kicker_id_;
  unsigned int shooter_id_;
  const std::vector<unsigned int>& receiver_ids_;
  std::shared_ptr<action::kick_action> kick_;
  std::vector<Eigen::Vector3d> positions;
};
} // agent
} // game
} // ai_server
#endif
