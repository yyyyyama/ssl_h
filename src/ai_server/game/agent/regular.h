#ifndef AI_SERVER_GAME_AGENT_REGULAR_H
#define AI_SERVER_GAME_AGENT_REGULAR_H

#include <queue>
#include <unordered_map>
#include "base.h"
#include "ai_server/game/action/chase_ball.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/game/action/marking.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/no_operation.h"

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
  // 構造体：IDと重要度
  struct id_importance_ {
    unsigned int id;
    double importance;

    bool operator<(const id_importance_& next) const {
      return importance < next.importance;
    }
  };

  const std::vector<unsigned int> ids_;
  bool is_chase_;
  unsigned int chase_id_;
  bool chase_finished_;
  bool kick_finished_;
  std::priority_queue<id_importance_> importance_list_;
  std::unordered_map<unsigned int, std::shared_ptr<action::marking>> ids_marking_;
  std::unordered_map<unsigned int, std::shared_ptr<action::no_operation>> ids_no_op_;
  std::vector<unsigned int> flow_ids_; // マーキング割り当ての際に余ったロボットID(=補欠)
  std::vector<unsigned int> no_op_ids_; // no_operationを割り当てられたID
  // Action
  std::shared_ptr<action::chase_ball> chase_ball_;
  std::shared_ptr<action::kick_action> kick_action_;

  // chase id_の候補を取得
  unsigned int get_chase_id();

  // 全て組み直す
  void reset_all();

  // マーキングの設定( change_all : マーキングの割当を全て変更する時はtrue)
  void set_marking(bool change_all);

  // 敵IDと重要度を設定、ソートした結果を返す
  std::priority_queue<id_importance_> get_importance_list();

  // ターゲットに最も近いロボットIDのイテレータを返す
  std::vector<unsigned int>::const_iterator nearest_id_itr(
      double target_x, double target_y, const std::vector<unsigned int>& can_ids) const;
};

} // agent
} // game
} // ai_server

#endif
