#ifndef AI_SERVER_GAME_ACTION_MARKING_H
#define AI_SERVER_GAME_ACTION_MARKING_H

#include "ai_server/model/command.h"
#include "ai_server/game/action/base.h"

namespace ai_server {
namespace game {
namespace action {
class marking : public base {
public:
  using base::base;
  enum class mark_mode { kick_block, shoot_block, corner_block };
  void mark_robot(unsigned int enemy_id);
  void set_mode(action::marking::mark_mode mode);
  void set_radius(double radius);
  model::command execute() override;
  bool finished() const override;

private:
  unsigned int enemy_id_;                  //指定された敵ロボットのidを保持
  mark_mode mode_ = mark_mode::kick_block; //マーキングの種類変更
  bool flag       = false;                 //終了判定のフラグ
  double radius_  = 250.0;
};
}
}
}
#endif
