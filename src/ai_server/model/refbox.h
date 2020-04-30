#ifndef AI_SERVER_MODEL_REFBOX_H
#define AI_SERVER_MODEL_REFBOX_H

#include <chrono>
#include <tuple>

#include "team_info.h"
#include "team_color.h"

#include "ssl-protos/refbox/referee.pb.h"

namespace ai_server {
namespace model {

class refbox {
public:
  enum class stage_name {
    normal_first_half_pre  = ssl_protos::refbox::Referee_Stage_NORMAL_FIRST_HALF_PRE,
    normal_first_half      = ssl_protos::refbox::Referee_Stage_NORMAL_FIRST_HALF,
    normal_half_time       = ssl_protos::refbox::Referee_Stage_NORMAL_HALF_TIME,
    normal_second_half_pre = ssl_protos::refbox::Referee_Stage_NORMAL_SECOND_HALF_PRE,
    normal_second_half     = ssl_protos::refbox::Referee_Stage_NORMAL_SECOND_HALF,
    extra_time_break       = ssl_protos::refbox::Referee_Stage_EXTRA_TIME_BREAK,
    extra_first_half_pre   = ssl_protos::refbox::Referee_Stage_EXTRA_FIRST_HALF_PRE,
    extra_first_half       = ssl_protos::refbox::Referee_Stage_EXTRA_FIRST_HALF,
    extra_half_time        = ssl_protos::refbox::Referee_Stage_EXTRA_HALF_TIME,
    extra_second_half_pre  = ssl_protos::refbox::Referee_Stage_EXTRA_SECOND_HALF_PRE,
    extra_second_half      = ssl_protos::refbox::Referee_Stage_EXTRA_SECOND_HALF,
    penalty_shootout_break = ssl_protos::refbox::Referee_Stage_PENALTY_SHOOTOUT_BREAK,
    penalty_shootout       = ssl_protos::refbox::Referee_Stage_PENALTY_SHOOTOUT,
    post_game              = ssl_protos::refbox::Referee_Stage_POST_GAME,
  };
  enum class game_command {
    halt                   = ssl_protos::refbox::Referee_Command_HALT,
    stop                   = ssl_protos::refbox::Referee_Command_STOP,
    normal_start           = ssl_protos::refbox::Referee_Command_NORMAL_START,
    force_start            = ssl_protos::refbox::Referee_Command_FORCE_START,
    prepare_kickoff_yellow = ssl_protos::refbox::Referee_Command_PREPARE_KICKOFF_YELLOW,
    prepare_kickoff_blue   = ssl_protos::refbox::Referee_Command_PREPARE_KICKOFF_BLUE,
    prepare_penalty_yellow = ssl_protos::refbox::Referee_Command_PREPARE_PENALTY_YELLOW,
    prepare_penalty_blue   = ssl_protos::refbox::Referee_Command_PREPARE_PENALTY_BLUE,
    direct_free_yellow     = ssl_protos::refbox::Referee_Command_DIRECT_FREE_YELLOW,
    direct_free_blue       = ssl_protos::refbox::Referee_Command_DIRECT_FREE_BLUE,
    indirect_free_yellow   = ssl_protos::refbox::Referee_Command_INDIRECT_FREE_YELLOW,
    indirect_free_blue     = ssl_protos::refbox::Referee_Command_INDIRECT_FREE_BLUE,
    timeout_yellow         = ssl_protos::refbox::Referee_Command_TIMEOUT_YELLOW,
    timeout_blue           = ssl_protos::refbox::Referee_Command_TIMEOUT_BLUE,
    goal_yellow            = ssl_protos::refbox::Referee_Command_GOAL_YELLOW,
    goal_blue              = ssl_protos::refbox::Referee_Command_GOAL_BLUE,
    ball_placement_yellow  = ssl_protos::refbox::Referee_Command_BALL_PLACEMENT_YELLOW,
    ball_placement_blue    = ssl_protos::refbox::Referee_Command_BALL_PLACEMENT_BLUE,
  };
  refbox();
  std::chrono::system_clock::time_point packet_timestamp() const;
  int stage_time_left() const;
  stage_name stage() const;
  game_command command() const;
  team_info team_yellow() const;
  team_info team_blue() const;
  std::tuple<double, double> ball_placement_position() const;
  std::optional<model::team_color> bot_substitution_by_team() const;
  void set_packet_timestamp(std::chrono::system_clock::time_point value);
  void set_stage_time_left(int value);
  void set_stage(stage_name value);
  void set_command(game_command value);
  void set_team_yellow(const team_info& value);
  void set_team_blue(const team_info& value);
  void set_ball_placement_position(std::tuple<double, double> value);
  void set_bot_substitution_by_team(std::optional<model::team_color> value);

private:
  std::chrono::system_clock::time_point packet_timestamp_;
  int stage_time_left_;
  stage_name stage_;
  game_command command_;
  team_info team_yellow_;
  team_info team_blue_;
  std::tuple<double, double> ball_placement_position_;
  std::optional<model::team_color> bot_substitution_by_team_;
};
} // namespace model
} // namespace ai_server

#endif
