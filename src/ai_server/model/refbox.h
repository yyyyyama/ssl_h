#ifndef AI_SERVER_MODEL_REFBOX_H
#define AI_SERVER_MODEL_REFBOX_H

#include <chrono>
#include <optional>
#include <Eigen/Geometry>

#include "team_info.h"
#include "team_color.h"

namespace ai_server {
namespace model {

class refbox {
public:
  enum class stage_name {
    normal_first_half_pre,
    normal_first_half,
    normal_half_time,
    normal_second_half_pre,
    normal_second_half,
    extra_time_break,
    extra_first_half_pre,
    extra_first_half,
    extra_half_time,
    extra_second_half_pre,
    extra_second_half,
    penalty_shootout_break,
    penalty_shootout,
    post_game,

    num_stages, //< stage_name の総数取得に使う
  };
  enum class game_command {
    halt,
    stop,
    normal_start,
    force_start,
    prepare_kickoff_yellow,
    prepare_kickoff_blue,
    prepare_penalty_yellow,
    prepare_penalty_blue,
    direct_free_yellow,
    direct_free_blue,
    indirect_free_yellow,
    indirect_free_blue,
    timeout_yellow,
    timeout_blue,
    ball_placement_yellow,
    ball_placement_blue,

    num_commands, //< game_command の総数取得に使う
  };
  refbox();
  std::chrono::system_clock::time_point packet_timestamp() const;
  int stage_time_left() const;
  stage_name stage() const;
  game_command command() const;
  team_info team_yellow() const;
  team_info team_blue() const;
  Eigen::Vector2d ball_placement_position() const;
  std::optional<model::team_color> bot_substitution_by_team() const;
  void set_packet_timestamp(std::chrono::system_clock::time_point value);
  void set_stage_time_left(int value);
  void set_stage(stage_name value);
  void set_command(game_command value);
  void set_team_yellow(const team_info& value);
  void set_team_blue(const team_info& value);
  void set_ball_placement_position(const Eigen::Vector2d& value);
  void set_bot_substitution_by_team(std::optional<model::team_color> value);

private:
  std::chrono::system_clock::time_point packet_timestamp_;
  int stage_time_left_;
  stage_name stage_;
  game_command command_;
  team_info team_yellow_;
  team_info team_blue_;
  Eigen::Vector2d ball_placement_position_;
  std::optional<model::team_color> bot_substitution_by_team_;
};

// @brief \p r から \p color の情報を取得する
inline auto our_team_info(const refbox& r, team_color color) {
  switch (color) {
    case team_color::blue:
      return r.team_blue();
    case team_color::yellow:
      return r.team_yellow();
  }
}

// @brief \p r から \p color の情報を取得する
inline auto enemy_team_info(const refbox& r, team_color color) {
  switch (color) {
    case team_color::blue:
      return r.team_yellow();
    case team_color::yellow:
      return r.team_blue();
  }
}

} // namespace model
} // namespace ai_server

#endif
