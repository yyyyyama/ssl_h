#ifndef AI_SERVER_GAME_AGENT_REGULAR_H
#define AI_SERVER_GAME_AGENT_REGULAR_H

#include "base.h"
#include "ai_server/game/action/chase_ball.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/game/action/marking.h"

namespace ai_server {
namespace game {
namespace agent {

class regular : public base {
public:
  regular(const model::world& world, bool is_yellow, const std::vector<unsigned int> ids);
  bool ball_chase() const;
  void set_ball_chase(bool ball_chase); // 全てのロボットをマーキングにする時はfalse
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  const std::vector<unsigned int> ids_;
  bool is_chase_;
  unsigned int chase_id_;
  bool chase_finished_;
  bool kick_finished_;
  std::vector<std::shared_ptr<action::marking>> mark_actions_;

  // マーキング相手を組み直す
  void set_marking(unsigned int& chase_ball_id, bool is_ball_chase);

  // ターゲットに最も近いロボットIDを返す(can_idsがconstになってない方は、can_idsから返り値の要素が削除される）
  unsigned int nearest_robot_id(double target_x, double target_y,
                                std::vector<unsigned int>& can_ids) const;
  unsigned int nearest_robot_id(double target_x, double target_y,
                                const std::vector<unsigned int>& can_ids) const;

  // IDと重要度
  struct id_importance_ {
    // ID, 重要度
    unsigned int id;
    double importance;

    bool operator<(const id_importance_& next) const {
      return importance > next.importance; //ソート基準=重要度が高い順
    }
  };

  // Action
  std::shared_ptr<action::marking> marking_;
  std::shared_ptr<action::chase_ball> chase_ball_;
  std::shared_ptr<action::kick_action> kick_action_;
};

} // agent
} // game
} // ai_server

#endif
