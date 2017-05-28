#ifndef AI_SERVER_GAME_AGENT_SETPLAY_H
#define AI_SERVER_GAME_AGENT_SETPLAY_H
#include <memory>
#include <vector>
#include <Eigen/Core>
#include "base.h"
#include "ai_server/model/world.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/game/action/receive.h"

namespace ai_server {
namespace game {
namespace agent {

class setplay : public base {
public:
  enum class state { finished, setup, pass, receive, shoot };
  enum class pos { near, mid, far };
  setplay(const model::world& world, bool is_yellow, unsigned int kicker_id,
          const std::vector<unsigned int>& receiver_id);

  void set_extra_robots(const std::vector<unsigned int>& ids);

  std::vector<std::shared_ptr<action::base>> execute() override;

  bool finished();

private:
  pos pos_;
  state state_ = state::setup;
  unsigned int kicker_id_;
  unsigned int shooter_id_;
  const std::vector<unsigned int>& receiver_ids_;
  Eigen::Vector2d ballv_;
  std::shared_ptr<action::kick_action> kick_;
  std::shared_ptr<action::receive> receive_;
  std::vector<std::shared_ptr<action::base>> baseaction_;
  Eigen::Vector2d passpos_;
  std::vector<Eigen::Vector2d> positions_;

  Eigen::Vector2d chooselocation(std::vector<Eigen::Vector2d> targets,
                                 model::world::robots_list enemy_robots, int dist = 1);
  double vectorangle(Eigen::Vector2d vec);
};
} // agent
} // game
} // ai_server
#endif
