#include "refbox.h"

namespace ai_server {
namespace model {

refbox::refbox() : team_yellow_("yellow"), team_blue_("blue") {
  packet_timestamp_ = 0;
  stage_time_left_  = 0;
  stage_            = refbox::stage_name::normal_first_half_pre;
  command_          = refbox::game_command::half;
}

int refbox::packet_timestamp() const {
  return packet_timestamp_;
}

int refbox::stage_time_left() const {
  return stage_time_left_;
}

refbox::stage_name refbox::stage() const {
  return stage_;
}

refbox::game_command refbox::command() const {
  return command_;
}

refbox::team_info refbox::team_yellow() const {
  return team_yellow_;
}

refbox::team_info refbox::team_blue() const {
  return team_blue_;
}

void refbox::set_packet_timestamp(int value) {
  packet_timestamp_ = value;
}

void refbox::set_stage_time_left(int value) {
  stage_time_left_ = value;
}

void refbox::set_stage(refbox::stage_name value) {
  stage_ = value;
}

void refbox::set_command(refbox::game_command value) {
  command_ = value;
}

void refbox::set_team_yellow(const refbox::team_info value) {
  team_yellow_ = value;
}

void refbox::set_team_blue(const refbox::team_info value) {
  team_blue_ = value;
}

refbox::team_info::team_info(std::string name) {
  name_              = name;
  score_             = 0;
  goalie_            = 0;
  red_cards_         = 0;
  yellow_cards_      = 0;
  yellow_card_times_ = 0;
  timeouts_          = 0;
  timeout_time_      = 0;
}

std::string refbox::team_info::name() const {
  return name_;
}

int refbox::team_info::score() const {
  return score_;
}

int refbox::team_info::goalie() const {
  return goalie_;
}

int refbox::team_info::red_cards() const {
  return red_cards_;
}

int refbox::team_info::yellow_cards() const {
  return yellow_cards_;
}

int refbox::team_info::yellow_card_times() const {
  return yellow_card_times_;
}

int refbox::team_info::timeouts() const {
  return timeouts_;
}

int refbox::team_info::timeout_time() const {
  return timeout_time_;
}

void refbox::team_info::set_score(int value) {
  score_ = value;
}

void refbox::team_info::set_goalie(int value) {
  goalie_ = value;
}

void refbox::team_info::set_red_cards(int value) {
  red_cards_ = value;
}

void refbox::team_info::set_yellow_cards(int value) {
  yellow_cards_ = value;
}

void refbox::team_info::set_yellow_card_times(int value) {
  yellow_card_times_ = value;
}

void refbox::team_info::set_timeouts(int value) {
  timeouts_ = value;
}

void refbox::team_info::set_timeout_time(int value) {
  timeout_time_ = value;
}
}
}
