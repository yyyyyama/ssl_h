#ifndef AI_SERVER_GAME_FORMATION_FIRST_FORMATION_H
#define AI_SERVER_GAME_FORMATION_FIRST_FORMATION_H

#include <array>
#include <chrono>
#include <memory>
#include <vector>
#include <boost/variant.hpp>
#include <Eigen/Dense>

#include "ai_server/game/agent/alignment.h"
#include "ai_server/game/agent/all.h"
#include "ai_server/game/agent/ball_placement.h"
#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/halt.h"
#include "ai_server/game/agent/kick_off.h"
#include "ai_server/game/agent/kick_off_waiter.h"
#include "ai_server/game/agent/marking.h"
#include "ai_server/game/agent/penalty_kick.h"
#include "ai_server/game/agent/setplay.h"
#include "ai_server/game/agent/stopgame.h"
#include "ai_server/game/formation/detail/convert_command.h"
#include "ai_server/model/refbox.h"

#include "base.h"

namespace ai_server::game::formation {

class first_formation : public base {
public:
  first_formation(context& ctx, const model::refbox& refcommand,
                  const std::vector<unsigned int>& ids);

  std::vector<std::shared_ptr<agent::base>> execute() override;

private:
  using id_vec            = std::vector<unsigned int>;
  using game_situation    = detail::game_situation;
  using game_command_type = detail::game_command_type;

  //キーパーのidをrefereeboxから来る値に更新する
  void update_keeper();
  //壁にするロボットを決定
  void decide_wall(const id_vec& visible_robots);
  //壁,キーパー,アタッカー以外でボールに一番近いロボットをキッカーに決定
  void decide_kicker(const id_vec& visible_robots);
  //キーパー,壁以外のロボットを抽出
  void extract_other_robots(const id_vec& visible_robots);
  //キーパー,壁,アタッカー,キッカー以外のロボットを抽出
  void extract_waiter();
  //キーパー以外のロボットを抽出
  void extract_except_keeper(const id_vec& visible_robots);

  //ボールを蹴ったか判定
  bool kicked(const Eigen::Vector2d& ball);

  //役割に対するid
  unsigned int keeper_;                           //キーパー
  id_vec wall_;                                   //壁
  unsigned int kicker_;                           //キッカー
  id_vec others_;                                 //キーパー,壁以外
  id_vec waiter_;                                 //キーパー,壁,キッカー以外
  id_vec except_keeper_;                          //キーパー以外
  constexpr static unsigned int shootout_id_ = 4; //シュートアウト
  unsigned long default_wall_count_;              //元の壁の数
  unsigned long wall_count_;                      //実際に設定する壁の数

  id_vec ids_;
  id_vec previous_visible_robots_;

  bool kicked_flag_;     //ボールが蹴られたか判断するためのフラグ
  bool initialize_flag_; // idが変わって初期化する必要があるとき

  std::chrono::steady_clock::time_point change_command_time_;
  std::chrono::steady_clock::time_point kicked_time_;
  bool time_over(const std::chrono::steady_clock::time_point& point,
                 const std::chrono::steady_clock::time_point& time, int count);

  void reset_agent();
  void role_reset(const id_vec& visible_robots);
  Eigen::Vector2d abp_target();
  Eigen::Vector2d prev_abp_target_;

  game_command_type previous_command_;
  game_command_type current_command_;
  //前回のgame_command_typeから変化したか判定する
  bool is_command_changed();

  Eigen::Vector2d previous_ball_;
  std::array<Eigen::Vector2d, 5> past_ball_;

  std::shared_ptr<agent::penalty_kick> pk_;
  std::shared_ptr<agent::halt> halt_;
  std::shared_ptr<agent::defense> df_;
  std::shared_ptr<agent::alignment> alignment_;
  std::shared_ptr<agent::kick_off> kickoff_;
  std::shared_ptr<agent::kick_off_waiter> kickoff_waiter_;
  std::shared_ptr<agent::stopgame> stop_;
  std::shared_ptr<agent::ball_placement> ball_placement_;
  std::shared_ptr<agent::all> all_;
  std::shared_ptr<agent::setplay> setplay_;
  std::shared_ptr<agent::all> shoot_out_;

  // agentを呼び出す関数
  std::shared_ptr<agent::penalty_kick> pk(bool start_flag, bool attack,
                                          unsigned int enemy_keeper);
  std::shared_ptr<agent::halt> halt();
  std::shared_ptr<agent::alignment> alignment();
  std::shared_ptr<agent::defense> defense(agent::defense::defense_mode mode);
  std::shared_ptr<agent::kick_off> kickoff(bool start_flag);
  std::shared_ptr<agent::kick_off_waiter> kickoff_waiter(
      agent::kick_off_waiter::kickoff_mode mode, bool attack);
  std::shared_ptr<agent::stopgame> stop();
  std::shared_ptr<agent::ball_placement> ball_placement(bool is_ally);
  std::shared_ptr<agent::all> all();
  std::shared_ptr<agent::setplay> setplay();
  std::shared_ptr<agent::all> shootout(bool start_flag, unsigned int enemy_keeper);
};
} // namespace ai_server::game::formation

#endif
