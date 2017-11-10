#include <limits>

#include "refbox.h"
#include "ssl-protos/refbox/referee.pb.h"

namespace ai_server {
namespace model {
namespace updater {

refbox::refbox() : refbox_{} {}

void refbox::update(const ssl_protos::refbox::Referee& referee) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);

  refbox_.set_packet_timestamp(referee.packet_timestamp());
  refbox_.set_stage_time_left(
      referee.has_stage_time_left()
          ? referee.stage_time_left()
          : std::numeric_limits<decltype(refbox_.stage_time_left())>::max());
  refbox_.set_stage(static_cast<model::refbox::stage_name>(referee.stage()));
  refbox_.set_command(static_cast<model::refbox::game_command>(referee.command()));

  auto to_team_info = [](auto&& team_info) {
    model::refbox::team_info result{team_info.name()};
    result.set_score(team_info.score());
    result.set_goalie(team_info.goalie());
    result.set_red_cards(team_info.red_cards());
    result.set_yellow_cards(team_info.yellow_cards());
    result.set_yellow_card_times(
        team_info.yellow_card_times_size() > 0 ? team_info.yellow_card_times(0) : 0);
    result.set_timeouts(team_info.timeouts());
    result.set_timeout_time(team_info.timeout_time());
    return result;
  };
  refbox_.set_team_blue(to_team_info(referee.blue()));
  refbox_.set_team_yellow(to_team_info(referee.yellow()));
}

model::refbox refbox::value() const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  return refbox_;
}

} // namespace updater
} // namespace model
} // namespace ai_server
