#ifndef AI_SERVER_GAME_CAPTAIN_FIRST_H
#define AI_SERVER_GAME_CAPTAIN_FIRST_H

#include <array>
#include <set>

#include <Eigen/Core>

#include "ai_server/logger/logger.h"

#include "base.h"
#include "detail/state.h"

namespace ai_server::game::captain {

class first : public base {
  using situation_type = detail::situation;

public:
  // @param ids 試合で使う機体の ID (キーパーを含む)
  first(context& ctx, const model::refbox& refbox, const std::set<unsigned int>& ids);

  std::shared_ptr<formation::v2::base> execute() override;

private:
  // 試合の状況に合わせて呼ばれる関数群

  void halt(situation_type situation, bool situation_changed);

  void stopgame(situation_type situation, bool situation_changed);

  void steady(situation_type situation, bool situation_changed);

  void kickoff_attack(situation_type situation, bool situation_changed);

  void kickoff_attack_start(situation_type situation, bool situation_changed);

  void kickoff_attack_start_to_steady(situation_type situation, bool situation_changed);

  void kickoff_defense(situation_type situation, bool siuation_changed);

  void kickoff_defense_to_steady(situation_type situation, bool situation_changed);

  void penalty_attack(situation_type situation, bool situation_changed);

  void penalty_attack_start(situation_type situation, bool situation_changed);

  void penalty_attack_start_to_steady(situation_type situation, bool situation_changed);

  void penalty_defense(situation_type situation, bool situation_changed);

  void penalty_defense_to_steady(situation_type situation, bool situation_changed);

  void shootout_attack_start(situation_type situation, bool situation_changed);

  void shootout_defense(situation_type situation, bool situation_changed);

  void shootout_defense_start(situation_type situation, bool situation_changed);

  void setplay_attack(situation_type situation, bool situation_changed);

  void setplay_attack_to_steady(situation_type situation, bool situation_changed);

  void setplay_defense(situation_type situation, bool situation_changed);

  void setplay_defense_to_steady(situation_type situation, bool situation_changed);

  void ball_placement(situation_type situation, bool situation_changed);

  void ball_placement_enemy(situation_type situation, bool situation_changed);

  void timeout(situation_type situation, bool situation_changed);

  /// デバッグ用
  /// 未実装の situation に入ったときに呼ばれ、warning を出す
  /// formation には halt が割り当てられる
  void undefined_event(situation_type situation, bool situation_changed);

private:
  /// 現在の状況に対応した関数ポインタを持つテーブル
  /// table_[初期化時であるか][situation]
  static constexpr auto table_ = [] {
    using handler_type            = void (first::*)(situation_type, bool);
    constexpr auto num_situations = static_cast<std::size_t>(situation_type::num_situations);

    std::array<std::array<handler_type, num_situations>, 2> table{};

    // 未実装の situation に入ったときには undefined_event が呼ばれるようにしておく
    // std::array<T, N>::fill は C++17 では constexpr じゃないので使えない
    for (auto& t : std::get<true>(table)) t = &first::undefined_event;
    for (auto& t : std::get<false>(table)) t = nullptr;

    // メンバ関数登録に使うヘルパ
    // s: 対象の situation
    // b: situation が切り替わったときの処理 --- true
    //    その後の更新時の処理               --- false
    auto on = [&table](situation_type s, bool b) -> handler_type& {
      return table.at(b).at(static_cast<std::size_t>(s));
    };

    on(situation_type::halt, true)                    = &first::halt;
    on(situation_type::stop, true)                    = &first::stopgame;
    on(situation_type::stop, false)                   = &first::stopgame;
    on(situation_type::force_start, true)             = &first::steady;
    on(situation_type::kickoff_attack, true)          = &first::kickoff_attack;
    on(situation_type::kickoff_attack_start, true)    = &first::kickoff_attack_start;
    on(situation_type::kickoff_attack_start, false)   = &first::kickoff_attack_start_to_steady;
    on(situation_type::kickoff_defense, true)         = &first::kickoff_defense;
    on(situation_type::kickoff_defense, false)        = &first::kickoff_defense_to_steady;
    on(situation_type::penalty_attack, true)          = &first::penalty_attack;
    on(situation_type::penalty_attack_start, true)    = &first::penalty_attack_start;
    on(situation_type::penalty_attack_start, false)   = &first::penalty_attack_start_to_steady;
    on(situation_type::penalty_defense, true)         = &first::penalty_defense;
    on(situation_type::penalty_defense, false)        = &first::penalty_defense_to_steady;
    on(situation_type::shootout_attack, true)         = &first::stopgame;
    on(situation_type::shootout_attack, false)        = &first::stopgame;
    on(situation_type::shootout_attack_start, true)   = &first::shootout_attack_start;
    on(situation_type::shootout_defense, true)        = &first::shootout_defense;
    on(situation_type::shootout_defense, false)       = &first::shootout_defense;
    on(situation_type::shootout_defense_start, true)  = &first::shootout_defense_start;
    on(situation_type::shootout_defense_start, false) = &first::shootout_defense_start;
    on(situation_type::setplay_attack, true)          = &first::setplay_attack;
    on(situation_type::setplay_attack, false)         = &first::setplay_attack_to_steady;
    on(situation_type::setplay_defense, true)         = &first::setplay_defense;
    on(situation_type::setplay_defense, false)        = &first::setplay_defense_to_steady;
    on(situation_type::ball_placement, true)          = &first::ball_placement;
    on(situation_type::ball_placement, false)         = &first::ball_placement;
    on(situation_type::ball_placement_enemy, true)    = &first::ball_placement_enemy;
    on(situation_type::ball_placement_enemy, false)   = &first::ball_placement_enemy;
    on(situation_type::timeout, true)                 = &first::timeout;

    return table;
  }();

  /// 試合で使う機体の ID (キーパーを含む)
  const std::set<unsigned int> ids_;

  detail::state state_;
  std::chrono::steady_clock::time_point situation_changed_time_;
  std::shared_ptr<formation::v2::base> current_formation_;
  Eigen::Vector2d prev_abp_target_;

  logger::logger_for<first> logger_;
};

} // namespace ai_server::game::captain

#endif // AI_SERVER_GAME_CAPTAIN_FIRST_H
