#ifndef AI_SERVER_GAME_CAPTAIN_DETAIL_STATE_H
#define AI_SERVER_GAME_CAPTAIN_DETAIL_STATE_H

#include <cstdint>

#include "ai_server/model/refbox.h"

namespace ai_server::game::captain::detail {

/// 試合の状況 (チームカラーに依存しない game_command)
enum class situation : std::size_t {
  halt,
  stop,

  force_start,

  kickoff_attack,
  kickoff_attack_start,
  kickoff_defense,

  penalty_attack,
  penalty_attack_start,
  penalty_defense,

  shootout_attack,
  shootout_attack_start,
  shootout_defense,
  shootout_defense_start,

  setplay_attack,
  setplay_defense,

  ball_placement,
  ball_placement_enemy,

  timeout,

  unknown, ///< 対応しないコマンドが渡された

  num_situations, ///< 状況の総数取得に使う
};

using stage_type   = model::refbox::stage_name;
using command_type = model::refbox::game_command;

/// refbox が出すコマンドから次の situation を生成するステートマシン
class state {
  situation current_situation_;
  situation (state::*update_)(stage_type stage, command_type command) noexcept;

  /// 内部状態の更新 (yellow 用)
  situation update_yellow(stage_type stage, command_type command) noexcept;
  /// 内部状態の更新 (blue 用)
  situation update_blue(stage_type stage, command_type command) noexcept;

public:
  /// @param color   チームカラー
  state(model::team_color color) noexcept
      : current_situation_{situation::unknown},
        update_{color == model::team_color::yellow ? &state::update_yellow
                                                   : &state::update_blue} {}

  /// @brief         refbox が出すコマンドから次の situation を生成する
  /// @param command refbox のコマンド
  situation update(stage_type stage, command_type command) noexcept {
    return (this->*update_)(stage, command);
  }

  /// @brief         現在の situation を取得する
  situation current_situation() const noexcept {
    return current_situation_;
  }
};

} // namespace ai_server::game::captain::detail

#endif // AI_SERVER_GAME_CAPTAIN_DETAIL_STATE_H
