#include "team_info.h"

namespace ai_server::model {

team_info::team_info(std::string name) {
  name_              = name;
  score_             = 0;
  goalie_            = 0;
  red_cards_         = 0;
  yellow_cards_      = 0;
  yellow_card_times_ = 0;
  timeouts_          = 0;
  timeout_time_      = 0;
  max_allowed_bots_  = 11;
}

std::string team_info::name() const {
  return name_;
}

int team_info::score() const {
  return score_;
}

unsigned int team_info::goalie() const {
  return goalie_;
}

int team_info::red_cards() const {
  return red_cards_;
}

int team_info::yellow_cards() const {
  return yellow_cards_;
}

int team_info::yellow_card_times() const {
  return yellow_card_times_;
}

int team_info::timeouts() const {
  return timeouts_;
}

int team_info::timeout_time() const {
  return timeout_time_;
}

int team_info::max_allowed_bots() const {
  return max_allowed_bots_;
}

void team_info::set_score(int value) {
  score_ = value;
}

void team_info::set_goalie(unsigned int value) {
  goalie_ = value;
}

void team_info::set_red_cards(int value) {
  red_cards_ = value;
}

void team_info::set_yellow_cards(int value) {
  yellow_cards_ = value;
}

void team_info::set_yellow_card_times(int value) {
  yellow_card_times_ = value;
}

void team_info::set_timeouts(int value) {
  timeouts_ = value;
}

void team_info::set_timeout_time(int value) {
  timeout_time_ = value;
}

void team_info::set_max_allowed_bots(int value) {
  max_allowed_bots_ = value;
}

} // namespace ai_server::model
