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
  void mark_robot(unsigned int enemy_id);
  void mark_mode(unsigned int mode);
  model::command execute();
  bool finished() const;

private:
  unsigned int enemy_id_; //指定された敵ロボットのidを保持
  unsigned int mode_ = 0;
  bool flag          = false; //終了判定のフラグ
};
}
}
}
#endif
