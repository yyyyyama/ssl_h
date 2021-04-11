#ifndef AI_SERVER_MODEL_REFMESSAGE_STRING_H
#define AI_SERVER_MODEL_REFMESSAGE_STRING_H

#include <string_view>

#include "refbox.h"

namespace ai_server::model {

constexpr std::string_view stage_name_to_string(model::refbox::stage_name s) noexcept {
  using namespace std::string_view_literals;
  switch (s) {
#define AI_SERVER_STAGE_NAME_ITEM(s) \
  case model::refbox::stage_name::s: \
    return #s##sv;

    AI_SERVER_STAGE_NAME_ITEM(normal_first_half_pre)
    AI_SERVER_STAGE_NAME_ITEM(normal_first_half)
    AI_SERVER_STAGE_NAME_ITEM(normal_half_time)
    AI_SERVER_STAGE_NAME_ITEM(normal_second_half_pre)
    AI_SERVER_STAGE_NAME_ITEM(normal_second_half)
    AI_SERVER_STAGE_NAME_ITEM(extra_time_break)
    AI_SERVER_STAGE_NAME_ITEM(extra_first_half_pre)
    AI_SERVER_STAGE_NAME_ITEM(extra_first_half)
    AI_SERVER_STAGE_NAME_ITEM(extra_half_time)
    AI_SERVER_STAGE_NAME_ITEM(extra_second_half_pre)
    AI_SERVER_STAGE_NAME_ITEM(extra_second_half)
    AI_SERVER_STAGE_NAME_ITEM(penalty_shootout_break)
    AI_SERVER_STAGE_NAME_ITEM(penalty_shootout)
    AI_SERVER_STAGE_NAME_ITEM(post_game)

#undef AI_SERVER_STAGE_NAME_ITEM

    default:
      return "[UNKNOWN]"sv;
  }
}

constexpr std::string_view game_command_to_string(model::refbox::game_command c) noexcept {
  using namespace std::string_view_literals;
  switch (c) {
#define AI_SERVER_GAME_COMMAND_ITEM(c) \
  case model::refbox::game_command::c: \
    return #c##sv;

    AI_SERVER_GAME_COMMAND_ITEM(halt)
    AI_SERVER_GAME_COMMAND_ITEM(stop)
    AI_SERVER_GAME_COMMAND_ITEM(normal_start)
    AI_SERVER_GAME_COMMAND_ITEM(force_start)
    AI_SERVER_GAME_COMMAND_ITEM(prepare_kickoff_yellow)
    AI_SERVER_GAME_COMMAND_ITEM(prepare_kickoff_blue)
    AI_SERVER_GAME_COMMAND_ITEM(prepare_penalty_yellow)
    AI_SERVER_GAME_COMMAND_ITEM(prepare_penalty_blue)
    AI_SERVER_GAME_COMMAND_ITEM(direct_free_yellow)
    AI_SERVER_GAME_COMMAND_ITEM(direct_free_blue)
    AI_SERVER_GAME_COMMAND_ITEM(indirect_free_yellow)
    AI_SERVER_GAME_COMMAND_ITEM(indirect_free_blue)
    AI_SERVER_GAME_COMMAND_ITEM(timeout_yellow)
    AI_SERVER_GAME_COMMAND_ITEM(timeout_blue)
    AI_SERVER_GAME_COMMAND_ITEM(ball_placement_yellow)
    AI_SERVER_GAME_COMMAND_ITEM(ball_placement_blue)

#undef AI_SERVER_GAME_COMMAND_ITEM

    default:
      return "[UNKNOWN]"sv;
  }
}

} // namespace ai_server::model

#endif // AI_SERVER_MODEL_REFMESSAGE_STRING_H
