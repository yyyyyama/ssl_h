#include "refbox.h"

namespace ai_server {
namespace model {

refbox::refbox()
    : team_yellow_("yellow"),
      team_blue_("blue"),
      ball_placement_position_{Eigen::Vector2d::Zero()} {
  stage_time_left_ = 0;
  stage_           = refbox::stage_name::normal_first_half_pre;
  command_         = refbox::game_command::halt;
}

std::chrono::system_clock::time_point refbox::packet_timestamp() const {
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

team_info refbox::team_yellow() const {
  return team_yellow_;
}

team_info refbox::team_blue() const {
  return team_blue_;
}

Eigen::Vector2d refbox::ball_placement_position() const {
  return ball_placement_position_;
}

std::optional<model::team_color> refbox::bot_substitution_by_team() const {
  return bot_substitution_by_team_;
}

void refbox::set_packet_timestamp(std::chrono::system_clock::time_point value) {
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

void refbox::set_team_yellow(const team_info& value) {
  team_yellow_ = value;
}

void refbox::set_team_blue(const team_info& value) {
  team_blue_ = value;
}

void refbox::set_ball_placement_position(const Eigen::Vector2d& value) {
  ball_placement_position_ = value;
}

void refbox::set_bot_substitution_by_team(std::optional<model::team_color> value) {
  bot_substitution_by_team_ = value;
}

} // namespace model
} // namespace ai_server
