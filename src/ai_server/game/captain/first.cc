#include <fmt/format.h>

#include "ai_server/game/formation/ball_placement.h"
#include "ai_server/game/formation/halt.h"
#include "ai_server/game/formation/setplay_defense.h"
#include "ai_server/game/formation/steady.h"
#include "ai_server/game/formation/stopgame.h"
#include "ai_server/game/formation/timeout.h"

#include "detail/situation_string.h"
#include "first.h"

namespace ai_server::game::captain {

first::first(context& ctx, const model::refbox& refbox, const std::set<unsigned int>& ids)
    : base{ctx, refbox}, ids_{ids}, state_{team_color()} {}

std::shared_ptr<formation::v2::base> first::execute() {
  // situation の更新
  const auto prev_situation    = state_.current_situation();
  const auto current_situation = state_.update(refbox().stage(), refbox().command());
  const auto situation_changed = current_situation != prev_situation;

  if (situation_changed) {
    logger_.debug(fmt::format("{} -> {}", situation_to_string(prev_situation),
                              situation_to_string(current_situation)));
  }

  // 状況に応じたメンバ関数を呼び出して formation を更新する
  if (auto f = table_.at(situation_changed).at(static_cast<std::size_t>(current_situation))) {
    (this->*f)(current_situation, situation_changed);
  }

  return current_formation_;
}

void first::halt([[maybe_unused]] situation_type situation,
                 [[maybe_unused]] bool situation_changed) {
  logger_.debug("halt");
  current_formation_ = make_formation<formation::halt>(std::vector(ids_.cbegin(), ids_.cend()));
}

void first::stopgame([[maybe_unused]] situation_type situation, bool situation_changed) {
  if (situation_changed) logger_.debug("stopgame");
  current_formation_ = make_formation<formation::stopgame>(
      std::vector(ids_.cbegin(), ids_.cend()), team_color() == model::team_color::yellow
                                                   ? refbox().team_yellow().goalie()
                                                   : refbox().team_blue().goalie());
}

void first::steady([[maybe_unused]] situation_type situation,
                   [[maybe_unused]] bool situation_changed) {
  logger_.debug("steady");
  current_formation_ = make_formation<formation::steady>(
      std::vector(ids_.cbegin(), ids_.cend()), team_color() == model::team_color::yellow
                                                   ? refbox().team_yellow().goalie()
                                                   : refbox().team_blue().goalie());
}

void first::setplay_defense([[maybe_unused]] situation_type situation,
                            [[maybe_unused]] bool situation_changed) {
  logger_.debug("setplay_defense");
  current_formation_ = make_formation<formation::setplay_defense>(
      std::vector(ids_.cbegin(), ids_.cend()), team_color() == model::team_color::yellow
                                                   ? refbox().team_yellow().goalie()
                                                   : refbox().team_blue().goalie());
}

void first::setplay_defense_to_steady([[maybe_unused]] situation_type situation,
                                      [[maybe_unused]] bool situation_changed) {
  if (auto f = std::dynamic_pointer_cast<formation::setplay_defense>(current_formation_)) {
    if (f->finished()) {
      logger_.debug("setplay_defense_to_steady");
      steady(situation, situation_changed);
    }
  }
}

void first::ball_placement(situation_type situation, bool situation_changed) {
  logger_.debug("ball_placement");
  const auto abp_target(refbox().ball_placement_position());
  if (!current_formation_ || situation_changed || abp_target != prev_abp_target_)
    current_formation_ = make_formation<formation::ball_placement>(
        std::vector(ids_.cbegin(), ids_.cend()), abp_target,
        situation == situation_type::ball_placement);
  prev_abp_target_ = abp_target;
}

void first::timeout([[maybe_unused]] situation_type situation,
                    [[maybe_unused]] bool situation_changed) {
  logger_.debug("timeout");
  current_formation_ =
      make_formation<formation::timeout>(std::vector(ids_.cbegin(), ids_.cend()));
}

void first::undefined_event(situation_type situation, bool situation_changed) {
  logger_.warn(fmt::format("undefined_event! ({})", situation_to_string(situation)));
  halt(situation, situation_changed);
}

} // namespace ai_server::game::captain
