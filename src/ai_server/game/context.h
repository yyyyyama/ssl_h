#ifndef AI_SERVER_GAME_CONTEXT_H
#define AI_SERVER_GAME_CONTEXT_H

#include <memory>

#include "ai_server/model/team_color.h"
#include "ai_server/model/world.h"

namespace ai_server::game {

class nnabla;

/// 戦略部全体で必要となる値
struct context {
  model::world world;
  model::team_color team_color;

  // game::context を利用する全ての箇所で NNabla のヘッダを include するのを防ぐため
  // また移行段階で nullptr を許容するため std::unique_ptr で扱う
  // この値に対する操作をする場合は "ai_server/game/nnabla.h" も include する
  std::unique_ptr<game::nnabla> nnabla;
};

} // namespace ai_server::game

#endif // AI_SERVER_GAME_CONTEXT_H
