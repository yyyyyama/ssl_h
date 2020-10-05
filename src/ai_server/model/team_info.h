#ifndef AI_SERVER_MODEL_TEAM_INFO_H
#define AI_SERVER_MODEL_TEAM_INFO_H

#include <string>

namespace ai_server::model {

class team_info {
  std::string name_;
  int score_;
  unsigned int goalie_;
  int red_cards_;
  int yellow_cards_;
  int yellow_card_times_;
  int timeouts_;
  int timeout_time_;
  int max_allowed_bots_;

public:
  team_info(std::string name);
  std::string name() const;
  int score() const;
  unsigned int goalie() const;
  int red_cards() const;
  int yellow_cards() const;
  int yellow_card_times() const;
  int timeouts() const;
  int timeout_time() const;
  int max_allowed_bots() const;
  void set_score(int value);
  void set_goalie(unsigned int value);
  void set_red_cards(int value);
  void set_yellow_cards(int value);
  void set_yellow_card_times(int value);
  void set_timeouts(int value);
  void set_timeout_time(int value);
  void set_max_allowed_bots(int value);
};

} // namespace ai_server::model

#endif // AI_SERVER_MODEL_TEAM_INFO_H
