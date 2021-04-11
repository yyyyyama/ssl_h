#include "ai_server/game/formation/detail/convert_command.h"

namespace ai_server::game::formation::detail {

game_command_type convert_command(model::team_color team_color,
                                  const game_command_type& previous_command,
                                  const model::refbox& refcommand) {
  using cmd            = ai_server::model::refbox::game_command;
  const auto command   = refcommand.command();
  const bool is_yellow = team_color == model::team_color::yellow;

  switch (command) {
    case cmd::halt:
      return std::make_tuple(game_situation::halt, false, false);

    case cmd::normal_start:
      switch (std::get<0>(previous_command)) {
        case game_situation::kickoff:
        case game_situation::penalty:
        case game_situation::shootout:
          return {std::get<0>(previous_command), std::get<1>(previous_command), true};
        default:
          return previous_command;
      }
      break;

    case cmd::force_start:
      return std::make_tuple(game_situation::force_start, true, false);
      break;

    case cmd::prepare_kickoff_yellow:
      return std::make_tuple(game_situation::kickoff, is_yellow, false);
      break;

    case cmd::prepare_kickoff_blue:
      return std::make_tuple(game_situation::kickoff, !is_yellow, false);
      break;

    case cmd::prepare_penalty_yellow:
      if (refcommand.stage() == model::refbox::stage_name::penalty_shootout) {
        return std::make_tuple(game_situation::shootout, is_yellow, false);
      } else {
        return std::make_tuple(game_situation::penalty, is_yellow, false);
      }
      break;

    case cmd::prepare_penalty_blue:
      if (refcommand.stage() == model::refbox::stage_name::penalty_shootout) {
        return std::make_tuple(game_situation::shootout, !is_yellow, false);
      } else {
        return std::make_tuple(game_situation::penalty, !is_yellow, false);
      }
      break;

    case cmd::direct_free_yellow:
    case cmd::indirect_free_yellow:
      return std::make_tuple(game_situation::setplay, is_yellow, false);

    case cmd::direct_free_blue:
    case cmd::indirect_free_blue:
      return std::make_tuple(game_situation::setplay, !is_yellow, false);

    case cmd::ball_placement_yellow:
      return std::make_tuple(game_situation::ball_placement, is_yellow, false);

    case cmd::ball_placement_blue:
      return std::make_tuple(game_situation::ball_placement, !is_yellow, false);

    case cmd::timeout_blue:
    case cmd::timeout_yellow:
      return std::make_tuple(game_situation::timeout, false, false);

    case cmd::stop:
      return std::make_tuple(game_situation::stop, false, false);
    default:
      return std::make_tuple(game_situation::stop, false, false);
  }
}
} // namespace ai_server::game::formation::detail
