#include "state.h"

namespace ai_server::game::captain::detail {

constexpr situation penalty(bool attack, bool shootout) noexcept {
  if (attack) {
    if (shootout)
      return situation::shootout_attack;
    else
      return situation::penalty_attack;
  } else {
    if (shootout)
      return situation::shootout_defense;
    else
      return situation::penalty_defense;
  }
}

template <model::team_color TeamColor>
constexpr situation next_situation(situation prev_situation, stage_type stage,
                                   command_type command) noexcept {
  switch (command) {
    case command_type::halt:
      return situation::halt;

    case command_type::force_start:
      return situation::force_start;

    case command_type::prepare_kickoff_yellow:
      if constexpr (TeamColor == model::team_color::yellow) {
        return situation::kickoff_attack;
      } else {
        return situation::kickoff_defense;
      }
    case command_type::prepare_kickoff_blue:
      if constexpr (TeamColor == model::team_color::blue) {
        return situation::kickoff_attack;
      } else {
        return situation::kickoff_defense;
      }

    case command_type::prepare_penalty_yellow:
      return penalty(TeamColor == model::team_color::yellow,
                     stage == stage_type::penalty_shootout);
    case command_type::prepare_penalty_blue:
      return penalty(TeamColor == model::team_color::blue,
                     stage == stage_type::penalty_shootout);

    case command_type::normal_start:
      switch (prev_situation) {
        case situation::kickoff_attack:
        case situation::kickoff_attack_start:
          return situation::kickoff_attack_start;

        case situation::penalty_attack:
        case situation::penalty_attack_start:
          return situation::penalty_attack_start;

        case situation::shootout_attack:
        case situation::shootout_attack_start:
          return situation::shootout_attack_start;

        case situation::shootout_defense:
        case situation::shootout_defense_start:
          return situation::shootout_defense_start;

        default:
          return prev_situation;
      }

    case command_type::direct_free_yellow:
    case command_type::indirect_free_yellow:
      if constexpr (TeamColor == model::team_color::yellow) {
        return situation::setplay_attack;
      } else {
        return situation::setplay_defense;
      }
    case command_type::direct_free_blue:
    case command_type::indirect_free_blue:
      if constexpr (TeamColor == model::team_color::blue) {
        return situation::setplay_attack;
      } else {
        return situation::setplay_defense;
      }

    case command_type::ball_placement_yellow:
      if constexpr (TeamColor == model::team_color::yellow) {
        return situation::ball_placement;
      } else {
        return situation::ball_placement_enemy;
      }
    case command_type::ball_placement_blue:
      if constexpr (TeamColor == model::team_color::blue) {
        return situation::ball_placement;
      } else {
        return situation::ball_placement_enemy;
      }

    case command_type::timeout_yellow:
    case command_type::timeout_blue:
      return situation::timeout;

    case command_type::stop:
      return situation::stop;

    default:
      return situation::unknown;
  }
}

situation state::update_blue(stage_type stage, command_type command) noexcept {
  current_situation_ =
      next_situation<model::team_color::blue>(current_situation_, stage, command);
  return current_situation_;
}

situation state::update_yellow(stage_type stage, command_type command) noexcept {
  current_situation_ =
      next_situation<model::team_color::yellow>(current_situation_, stage, command);
  return current_situation_;
}

} // namespace ai_server::game::captain::detail
