#ifndef AI_SERVER_GAME_AGENT_STOPGAME_H
#define AI_SERVER_GAME_AGENT_STOPGAME_H
#include <memory>
#include <vector>
#include "ai_server/game/agent/base.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/game/action/receive.h"
#include "ai_server/model/world.h"
#include <Eigen/Dense>

namespace ai_server {
namespace game {
namespace agent {

class stopgame : public base {
private:
  const std::vector<unsigned int> ids_;
  const std::vector<unsigned int> init_ids_;
  unsigned int nearest_robot_;
  unsigned int receiver_robot_;

  unsigned int init_ids_size_;

  bool is_placement_;
  bool abp_flag_;
  bool kick_abp_flag_;
  bool abp_start_;
  bool wait_flag_;
  bool before_kick_finished_;
  bool kick_failed_;
  double pass_dist_;
  double begin_dist_;
  Eigen::Vector2d abp_target_;
  Eigen::Vector2d receiverxy_;
  util::time_point_type begin_;
  util::time_point_type now_;
  util::time_point_type kick_time_;
  std::shared_ptr<action::base> abp_;
  std::shared_ptr<action::base> pull_;
  std::shared_ptr<action::receive> receive_;
  std::shared_ptr<action::kick_action> kick_;

public:
  stopgame(const model::world& world, bool is_yellow, const std::vector<unsigned int>& ids);
  stopgame(const model::world& world, bool is_yellow, const std::vector<unsigned int>& ids,
           Eigen::Vector2d abp_target, bool abp_flag);
  std::vector<std::shared_ptr<action::base>> execute() override;
};

} // namespace agent
} // namespace game
} // namespace ai_server
#endif
