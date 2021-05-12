#include <chrono>
#include <limits>

#include "ai_server/util/math/affine.h"
#include "refbox.h"
#include "ssl-protos/gc_referee_message.pb.h"

namespace ai_server {
namespace model {
namespace updater {

constexpr model::refbox::stage_name to_stage_name(ssl_protos::gc::Referee_Stage s) {
  switch (s) {
    case ssl_protos::gc::Referee_Stage_NORMAL_FIRST_HALF_PRE:
      return model::refbox::stage_name::normal_first_half_pre;
    case ssl_protos::gc::Referee_Stage_NORMAL_FIRST_HALF:
      return model::refbox::stage_name::normal_first_half;
    case ssl_protos::gc::Referee_Stage_NORMAL_HALF_TIME:
      return model::refbox::stage_name::normal_half_time;
    case ssl_protos::gc::Referee_Stage_NORMAL_SECOND_HALF_PRE:
      return model::refbox::stage_name::normal_second_half_pre;
    case ssl_protos::gc::Referee_Stage_NORMAL_SECOND_HALF:
      return model::refbox::stage_name::normal_second_half;
    case ssl_protos::gc::Referee_Stage_EXTRA_TIME_BREAK:
      return model::refbox::stage_name::extra_time_break;
    case ssl_protos::gc::Referee_Stage_EXTRA_FIRST_HALF_PRE:
      return model::refbox::stage_name::extra_first_half_pre;
    case ssl_protos::gc::Referee_Stage_EXTRA_FIRST_HALF:
      return model::refbox::stage_name::extra_first_half;
    case ssl_protos::gc::Referee_Stage_EXTRA_HALF_TIME:
      return model::refbox::stage_name::extra_half_time;
    case ssl_protos::gc::Referee_Stage_EXTRA_SECOND_HALF_PRE:
      return model::refbox::stage_name::extra_second_half_pre;
    case ssl_protos::gc::Referee_Stage_EXTRA_SECOND_HALF:
      return model::refbox::stage_name::extra_second_half;
    case ssl_protos::gc::Referee_Stage_PENALTY_SHOOTOUT_BREAK:
      return model::refbox::stage_name::penalty_shootout_break;
    case ssl_protos::gc::Referee_Stage_PENALTY_SHOOTOUT:
      return model::refbox::stage_name::penalty_shootout;
    case ssl_protos::gc::Referee_Stage_POST_GAME:
      return model::refbox::stage_name::post_game;
  }
}

constexpr model::refbox::game_command to_game_command(ssl_protos::gc::Referee_Command c) {
  switch (c) {
    case ssl_protos::gc::Referee_Command_HALT:
      return model::refbox::game_command::halt;
    case ssl_protos::gc::Referee_Command_STOP:
      return model::refbox::game_command::stop;
    case ssl_protos::gc::Referee_Command_NORMAL_START:
      return model::refbox::game_command::normal_start;
    case ssl_protos::gc::Referee_Command_FORCE_START:
      return model::refbox::game_command::force_start;
    case ssl_protos::gc::Referee_Command_PREPARE_KICKOFF_YELLOW:
      return model::refbox::game_command::prepare_kickoff_yellow;
    case ssl_protos::gc::Referee_Command_PREPARE_KICKOFF_BLUE:
      return model::refbox::game_command::prepare_kickoff_blue;
    case ssl_protos::gc::Referee_Command_PREPARE_PENALTY_YELLOW:
      return model::refbox::game_command::prepare_penalty_yellow;
    case ssl_protos::gc::Referee_Command_PREPARE_PENALTY_BLUE:
      return model::refbox::game_command::prepare_penalty_blue;
    case ssl_protos::gc::Referee_Command_DIRECT_FREE_YELLOW:
      return model::refbox::game_command::direct_free_yellow;
    case ssl_protos::gc::Referee_Command_DIRECT_FREE_BLUE:
      return model::refbox::game_command::direct_free_blue;
    case ssl_protos::gc::Referee_Command_INDIRECT_FREE_YELLOW:
      return model::refbox::game_command::indirect_free_yellow;
    case ssl_protos::gc::Referee_Command_INDIRECT_FREE_BLUE:
      return model::refbox::game_command::indirect_free_blue;
    case ssl_protos::gc::Referee_Command_TIMEOUT_YELLOW:
      return model::refbox::game_command::timeout_yellow;
    case ssl_protos::gc::Referee_Command_TIMEOUT_BLUE:
      return model::refbox::game_command::timeout_blue;
    case ssl_protos::gc::Referee_Command_BALL_PLACEMENT_YELLOW:
      return model::refbox::game_command::ball_placement_yellow;
    case ssl_protos::gc::Referee_Command_BALL_PLACEMENT_BLUE:
      return model::refbox::game_command::ball_placement_blue;
  }
}

refbox::refbox() : refbox_{}, affine_{Eigen::Translation3d{.0, .0, .0}} {}

void refbox::update(const ssl_protos::gc::Referee& referee) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  refbox_.set_packet_timestamp(std::chrono::system_clock::time_point{
      std::chrono::microseconds{referee.packet_timestamp()}});
  refbox_.set_stage_time_left(
      referee.has_stage_time_left()
          ? referee.stage_time_left()
          : std::numeric_limits<decltype(refbox_.stage_time_left())>::max());
  refbox_.set_stage(to_stage_name(referee.stage()));
  refbox_.set_command(to_game_command(referee.command()));

  auto to_team_info = [](auto&& team_info) {
    model::team_info result{team_info.name()};
    result.set_score(team_info.score());
    result.set_goalie(team_info.goalkeeper());
    result.set_red_cards(team_info.red_cards());
    result.set_yellow_cards(team_info.yellow_cards());
    result.set_yellow_card_times(
        team_info.yellow_card_times_size() > 0 ? team_info.yellow_card_times(0) : 0);
    result.set_timeouts(team_info.timeouts());
    result.set_timeout_time(team_info.timeout_time());
    result.set_max_allowed_bots(team_info.max_allowed_bots());
    return result;
  };
  refbox_.set_team_blue(to_team_info(referee.blue()));
  refbox_.set_team_yellow(to_team_info(referee.yellow()));

  if (referee.has_designated_position()) {
    const auto& dp = referee.designated_position();
    const Eigen::Vector2d p(dp.x(), dp.y());
    refbox_.set_ball_placement_position(util::math::transform(affine_, p));
  }

  const auto& events = referee.game_events();
  const auto result  = std::find_if(events.cbegin(), events.cend(),
                                   [](const auto& e) { return e.has_bot_substitution(); });

  if (result != events.end()) {
    const auto& bs = result->bot_substitution();
    switch (bs.by_team()) {
      case ssl_protos::gc::YELLOW:
        refbox_.set_bot_substitution_by_team(team_color::yellow);
        break;
      case ssl_protos::gc::BLUE:
        refbox_.set_bot_substitution_by_team(team_color::blue);
        break;
      case ssl_protos::gc::UNKNOWN:
        refbox_.set_bot_substitution_by_team(std::nullopt);
        break;
    }
  } else {
    refbox_.set_bot_substitution_by_team(std::nullopt);
  }
}

void refbox::set_transformation_matrix(const Eigen::Affine3d& matrix) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  affine_ = matrix;
}

model::refbox refbox::value() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return refbox_;
}

} // namespace updater
} // namespace model
} // namespace ai_server
