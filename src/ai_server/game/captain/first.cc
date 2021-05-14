#include <fmt/format.h>

#include "ai_server/game/formation/ball_placement.h"
#include "ai_server/game/formation/halt.h"
#include "ai_server/game/formation/kickoff_attack.h"
#include "ai_server/game/formation/kickoff_defense.h"
#include "ai_server/game/formation/penalty_attack.h"
#include "ai_server/game/formation/penalty_defense.h"
#include "ai_server/game/formation/setplay_attack.h"
#include "ai_server/game/formation/setplay_defense.h"
#include "ai_server/game/formation/shootout_attack.h"
#include "ai_server/game/formation/shootout_defense.h"
#include "ai_server/game/formation/steady.h"
#include "ai_server/game/formation/stopgame.h"
#include "ai_server/game/formation/timeout.h"

#include "detail/situation_string.h"
#include "first.h"

namespace ai_server::game::captain {
using namespace std::chrono_literals;

first::first(context& ctx, const model::refbox& refbox, const std::set<unsigned int>& ids)
    : base{ctx, refbox},
      ids_{ids},
      state_{team_color()},
      situation_changed_time_{std::chrono::steady_clock::time_point::min()} {}

std::shared_ptr<formation::v2::base> first::execute() {
  // situation の更新
  const auto prev_situation    = state_.current_situation();
  const auto current_situation = state_.update(refbox().stage(), refbox().command());
  const auto situation_changed = current_situation != prev_situation;

  if (situation_changed) {
    logger_.debug(fmt::format("{} -> {}", situation_to_string(prev_situation),
                              situation_to_string(current_situation)));
    situation_changed_time_ = std::chrono::steady_clock::now();
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
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie());
}

void first::steady([[maybe_unused]] situation_type situation,
                   [[maybe_unused]] bool situation_changed) {
  logger_.debug("steady");
  current_formation_ =
      make_formation<formation::steady>(std::vector(ids_.cbegin(), ids_.cend()),
                                        model::our_team_info(refbox(), team_color()).goalie());
}

void first::kickoff_attack([[maybe_unused]] situation_type situation,
                           [[maybe_unused]] bool situation_changed) {
  logger_.debug("kickoff_attack");
  current_formation_ = make_formation<formation::kickoff_attack>(
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie(), false);
}

void first::kickoff_attack_start([[maybe_unused]] situation_type situation,
                                 [[maybe_unused]] bool situation_changed) {
  logger_.debug("kickoff_attack_start");
  current_formation_ = make_formation<formation::kickoff_attack>(
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie(), true);
}

void first::kickoff_attack_start_to_steady(situation_type situation, bool situation_changed) {
  if (auto f = std::dynamic_pointer_cast<formation::kickoff_attack>(current_formation_)) {
    if (f->finished() || std::chrono::steady_clock::now() - situation_changed_time_ > 8s) {
      logger_.debug("kickoff_attack_start -> steady");
      steady(situation, situation_changed);
    }
  }
}

void first::kickoff_defense([[maybe_unused]] situation_type situation,
                            [[maybe_unused]] bool situation_changed) {
  logger_.debug("kickoff_defense");
  current_formation_ = make_formation<formation::kickoff_defense>(
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie());
}

void first::kickoff_defense_to_steady(situation_type situation, bool situation_changed) {
  if (auto f = std::dynamic_pointer_cast<formation::kickoff_defense>(current_formation_)) {
    if (f->finished()) {
      logger_.debug("kickoff_defense_to_steady");
      steady(situation, situation_changed);
    }
  }
}

void first::penalty_attack([[maybe_unused]] situation_type situation,
                           [[maybe_unused]] bool situation_changed) {
  logger_.debug("penalty_attack");
  current_formation_ = make_formation<formation::penalty_attack>(
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie(),
      model::enemy_team_info(refbox(), team_color()).goalie(), false);
}

void first::penalty_attack_start([[maybe_unused]] situation_type situation,
                                 [[maybe_unused]] bool situation_changed) {
  logger_.debug("penalty_attack_start");
  current_formation_ = make_formation<formation::penalty_attack>(
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie(),
      model::enemy_team_info(refbox(), team_color()).goalie(), true);
}

void first::penalty_attack_start_to_steady(situation_type situation, bool situation_changed) {
  if (auto f = std::dynamic_pointer_cast<formation::penalty_attack>(current_formation_)) {
    if (f->finished() || std::chrono::steady_clock::now() - situation_changed_time_ > 10s) {
      logger_.debug("penalty_attack_start -> steady");
      steady(situation, situation_changed);
    }
  }
}

void first::penalty_defense([[maybe_unused]] situation_type situation,
                            [[maybe_unused]] bool situation_changed) {
  logger_.debug("penalty_defense");
  current_formation_ = make_formation<formation::penalty_defense>(
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie(),
      model::enemy_team_info(refbox(), team_color()).goalie());
}

void first::penalty_defense_to_steady(situation_type situation, bool situation_changed) {
  if (auto f = std::dynamic_pointer_cast<formation::penalty_defense>(current_formation_)) {
    if (f->finished()) {
      logger_.debug("penalty_defense -> steady");
      steady(situation, situation_changed);
    }
  }
}

void first::shootout_attack_start([[maybe_unused]] situation_type situation,
                                  [[maybe_unused]] bool situation_changed) {
  logger_.debug("shootout_attack_start");
  current_formation_ = make_formation<formation::shootout_attack>(
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie());
}

void first::shootout_defense([[maybe_unused]] situation_type situation,
                             [[maybe_unused]] bool situation_changed) {
  logger_.debug("shootout_defense");
  current_formation_ = make_formation<formation::shootout_defense>(
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie(),
      model::enemy_team_info(refbox(), team_color()).goalie());
}

void first::shootout_defense_start([[maybe_unused]] situation_type situation,
                                   [[maybe_unused]] bool situation_changed) {
  if (auto f = std::dynamic_pointer_cast<formation::shootout_defense>(current_formation_)) {
    logger_.debug("shootout_defense_start");
    f->start();
  }
}

void first::setplay_attack([[maybe_unused]] situation_type situation,
                           [[maybe_unused]] bool situation_changed) {
  logger_.debug("setplay_attack");
  current_formation_ = make_formation<formation::setplay_attack>(
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie());
}

void first::setplay_attack_to_steady(situation_type situation, bool situation_changed) {
  if (auto f = std::dynamic_pointer_cast<formation::setplay_attack>(current_formation_)) {
    if (f->finished() || std::chrono::steady_clock::now() - situation_changed_time_ > 15s) {
      logger_.debug("setplay_attack -> steady");
      steady(situation, situation_changed);
    }
  }
}

void first::setplay_defense([[maybe_unused]] situation_type situation,
                            [[maybe_unused]] bool situation_changed) {
  logger_.debug("setplay_defense");
  current_formation_ = make_formation<formation::setplay_defense>(
      std::vector(ids_.cbegin(), ids_.cend()),
      model::our_team_info(refbox(), team_color()).goalie());
}

void first::setplay_defense_to_steady(situation_type situation, bool situation_changed) {
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

void first::ball_placement_enemy([[maybe_unused]] situation_type situation,
                                 bool situation_changed) {
  logger_.debug("ball_placement_enemy");
  const auto abp_target(refbox().ball_placement_position());
  if (!current_formation_ || situation_changed || abp_target != prev_abp_target_) {
    current_formation_ = make_formation<formation::ball_placement>(
        std::vector(ids_.cbegin(), ids_.cend()), abp_target, false);
  }
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
