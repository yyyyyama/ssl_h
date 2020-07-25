#ifndef AI_SERVER_GAME_CAPTAIN_DETAIL_SITUATION_STRING_H
#define AI_SERVER_GAME_CAPTAIN_DETAIL_SITUATION_STRING_H

#include <string_view>

#include "state.h"

namespace ai_server::game::captain::detail {

constexpr std::string_view situation_to_string(situation s) {
  using namespace std::string_view_literals;

  switch (s) {
#define AI_SERVER_SITUATION_ITEM(s) \
  case s:                           \
    return #s##sv;

    AI_SERVER_SITUATION_ITEM(situation::halt)
    AI_SERVER_SITUATION_ITEM(situation::stop)
    AI_SERVER_SITUATION_ITEM(situation::force_start)
    AI_SERVER_SITUATION_ITEM(situation::kickoff_attack)
    AI_SERVER_SITUATION_ITEM(situation::kickoff_attack_start)
    AI_SERVER_SITUATION_ITEM(situation::kickoff_defense)
    AI_SERVER_SITUATION_ITEM(situation::penalty_attack)
    AI_SERVER_SITUATION_ITEM(situation::penalty_attack_start)
    AI_SERVER_SITUATION_ITEM(situation::penalty_defense)
    AI_SERVER_SITUATION_ITEM(situation::shootout_attack)
    AI_SERVER_SITUATION_ITEM(situation::shootout_attack_start)
    AI_SERVER_SITUATION_ITEM(situation::shootout_defense)
    AI_SERVER_SITUATION_ITEM(situation::shootout_defense_start)
    AI_SERVER_SITUATION_ITEM(situation::setplay_attack)
    AI_SERVER_SITUATION_ITEM(situation::setplay_defense)
    AI_SERVER_SITUATION_ITEM(situation::ball_placement)
    AI_SERVER_SITUATION_ITEM(situation::ball_placement_enemy)
    AI_SERVER_SITUATION_ITEM(situation::timeout)
    AI_SERVER_SITUATION_ITEM(situation::unknown)
    AI_SERVER_SITUATION_ITEM(situation::num_situations)

#undef AI_SERVER_SITUATION_ITEM

    default:
      return "[UNKNOWN]"sv;
  }
}

} // namespace ai_server::game::captain::detail

#endif // AI_SERVER_GAME_CAPTAIN_DETAIL_SITUATION_STRING_H
