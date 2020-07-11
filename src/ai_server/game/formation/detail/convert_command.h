#ifndef AI_SERVER_GAME_FORMATION_DETAIL_CONVERT_COMMAND_H
#define AI_SERVER_GAME_FORMATION_DETAIL_CONVERT_COMMAND_H

#include "ai_server/model/refbox.h"

namespace ai_server::game::formation::detail {

enum class game_situation {
  halt,
  stop,
  kickoff,
  penalty,
  force_start,
  setplay,
  shootout,
  ball_placement,
  timeout
};

// 試合の状態，攻撃モードかどうか，スタートしたかどうか
using game_command_type = std::tuple<game_situation, bool, bool>;

//コマンドを使いやすい形に変形
game_command_type convert_command(model::team_color team_color,
                                  const game_command_type& previous_command,
                                  const model::refbox& refcommand);

} // namespace ai_server::game::formation::detail

#endif
