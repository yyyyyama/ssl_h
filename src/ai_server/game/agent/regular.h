#ifndef AI_SERVER_GAME_AGENT_REGULAR_H
#define AI_SERVER_GAME_AGENT_REGULAR_H

#include "base.h"
#include "ai_server/game/action/marking.h"
#include "ai_server/game/action/move.h" //debug
//#include "ai_server/game/action/chase_ball.h"
#include "ai_server/model/robot.h"

namespace ai_server {
namespace game {
namespace agent {

class regular : public base {
public:
  regular(const model::world& world, bool is_yellow, const std::vector<unsigned int>& ids);

  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  const std::vector<unsigned int>& ids_;

  //ターゲットに最も近いロボットIDを返す
  unsigned int nearest_robot_id(double target_x, double target_y,
                                std::vector<unsigned int>& can_ids);

  //敵ロボIDと、対応の優先度構造体
  //重要度の所には、「(条件1)*a+(条件2)*b
  //(a,bは任意の数)」を代入し、重要度の高い順にロボットが対応する。a,bには条件の優先度合いを指定する。
  struct that_robot_importance_ {
    unsigned int id;   // ID
    double importance; //重要度

    //ソート基準を重要度の方に設定
    bool operator<(const that_robot_importance_& next) const {
      return importance > next.importance;
    }
  };

  // Action
  std::shared_ptr<action::marking> marking_;
  // std::shared_ptr<action::chase_ball> chase_ball_;
};

} // agent
} // game
} // ai_server

#endif
