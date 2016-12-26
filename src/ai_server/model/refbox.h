#include <string>
#ifndef INCLUDE_GUARD_REFBOX
#define INCLUDE_GUARD_REFBOX

namespace ai_server {
namespace model {

class refbox {
  class team_info {
    std::string name_;
    int score_;
    int goalie_;
    int red_cards_;
    int yellow_cards_;
    int yellow_card_times_;
    int timeouts_;
    int timeout_time_;

  public:
    team_info(std::string name);
    std::string name() const;
    int score() const;
    int goalie() const;
    int red_cards() const;
    int yellow_cards() const;
    int yellow_card_times() const;
    int timeouts() const;
    int timeout_time() const;
    void set_score(int value);
    void set_goalie(int value);
    void set_red_cards(int value);
    void set_yellow_cards(int value);
    void set_yellow_card_times(int value);
    void set_timeouts(int value);
    void set_timeout_time(int value);
  };

  enum class stage_name {
    normal_first_half_pre,
    normal_first_half,
    normal_half_time,
    normal_second_half_pre,
    normal_second_half,
    extra_time_brake,
    extra_first_half_pre,
    extra_first_half,
    extra_half_time,
    extra_second_half_pre,
    extra_second_half,
    penalty_shootout_break,
    penalty_shootout,
    post_game
  };
  enum class game_command {
    half,
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
    goal_yellow,
    goal_blue,
    ball_placement_yellow,
    ball_placement_blue
  };
  const int normal_first_half_pre = 0;
  const int half = 0;
  int packet_timestamp_;
  int stage_time_left_;
  stage_name stage_;
  game_command command_;
  team_info team_yellow_;
  team_info team_blue_;

public:
  refbox();
  int packet_timestamp() const;
  int stage_time_left() const;
  stage_name stage() const;
  game_command command() const;
  team_info team_yellow() const;
  team_info team_blue() const;
  void set_packet_timestamp(int value);
  void set_stage_time_left(int value);
  void set_stage(stage_name value);
  void set_command(game_command value);
  void set_team_yellow(team_info value);
  void set_team_blue(team_info value);
};
}
}

#endif

