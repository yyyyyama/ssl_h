#ifndef AI_SERVER_GAME_FORMATION_FIRST_FORMATION_H
#define AI_SERVER_GAME_FORMATION_FIRST_FORMATION_H

#include <memory>
#include <vector>
#include <chrono>
#include <Eigen/Dense>

#include "base.h"
#include "ai_server/model/refbox.h"
#include "ai_server/game/agent/ball_placement.h"
#include "ai_server/game/agent/defense.h"
#include "ai_server/game/agent/halt.h"
#include "ai_server/game/agent/kick_off.h"
#include "ai_server/game/agent/kick_off_waiter.h"
#include "ai_server/game/agent/penalty_kick.h"
#include "ai_server/game/agent/regular.h"
#include "ai_server/game/agent/setplay.h"
#include "ai_server/game/agent/stopgame.h"

namespace ai_server {
namespace game {
namespace formation {

class first_formation : public base {
public:
  enum class command {
    halt,
    stop,
    kickoff_attack_start,
    penalty_attack_start,
    force_start,
    kickoff_attack,
    kickoff_defense,
    penalty_attack,
    penalty_defense,
    setplay_attack,
    setplay_defense,
    shootout_attack,
    shootout_attack_start,
    shootout_defense,
    shootout_defense_start,
    ball_placement,
    ball_placement_enemy
  };

  first_formation(context& ctx, const model::refbox& refcommand,
                  const std::vector<unsigned int>& ids);

  std::vector<std::shared_ptr<agent::base>> execute() override;

private:
  void update_keeper(); //キーパーのidをrefereeboxから来る値に更新する
  void decide_wall_count(const std::vector<unsigned int>& visible_robots); //壁の数を決定
  void decide_wall(const std::vector<unsigned int>& visible_robots); //壁にするロボットを決定
  void decide_kicker(
      const std::vector<unsigned int>&
          visible_robots); //壁,キーパー以外でボールに一番近いロボットをキッカーに決定
  void extract_other_robots(
      const std::vector<unsigned int>& visible_robots); //キーパー,壁以外のロボットを抽出
  void extract_waiter(); //キーパー,壁,キッカー以外のロボットを抽出
  void extract_except_keeper(
      const std::vector<unsigned int>& visible_robots); //キーパー以外のロボットを抽出

  //コマンドを使いやすい形に変形
  command convert_command(model::refbox::game_command command);
  //ボールを蹴ったか判定
  bool kicked(model::ball ball);

  //役割に対するid
  unsigned int keeper_;                     //キーパー
  std::vector<unsigned int> wall_;          //壁
  unsigned int kicker_;                     //キッカー
  std::vector<unsigned int> others_;        //キーパー,壁以外
  std::vector<unsigned int> waiter_;        //キーパー,壁,キッカー以外
  std::vector<unsigned int> except_keeper_; //キーパー以外
  unsigned long wall_count_;                //壁の数

  std::vector<unsigned int> ids_;
  std::vector<unsigned int> previous_visible_robots_;

  bool kicked_flag_;     //ボールが蹴られたか判断するためのフラグ
  bool initialize_flag_; // idが変わって初期化する必要があるとき
  bool regular_flag_; //定常状態に遷移した際に、一回だけagentを初期化したいため必要
  bool is_corner_;

  std::chrono::high_resolution_clock::time_point change_command_time_;
  bool time_over(std::chrono::high_resolution_clock::time_point point, int count);

  void reset_agent();
  void role_reset(const std::vector<unsigned int>& visible_robots);
  Eigen::Vector2d abp_target() const;
  Eigen::Vector2d prev_abp_target_;

  model::refbox::game_command previous_refcommand_;
  command previous_command_;
  command current_command_;
  //前回のcommandから変化したか判定する
  bool is_command_changed();

  model::ball previous_ball_;
  std::vector<model::ball> past_ball_;

  std::shared_ptr<agent::penalty_kick> pk_;
  std::shared_ptr<agent::halt> halt_;
  std::shared_ptr<agent::defense> df_;
  std::shared_ptr<agent::kick_off> kickoff_;
  std::shared_ptr<agent::kick_off_waiter> kickoff_waiter_;
  std::shared_ptr<agent::stopgame> stop_;
  std::shared_ptr<agent::ball_placement> ball_placement_;
  std::shared_ptr<agent::regular> regular_;
  std::shared_ptr<agent::setplay> setplay_;
  std::shared_ptr<agent::penalty_kick> shoot_out_;

  // agentを呼び出す関数
  std::shared_ptr<agent::penalty_kick> pk(bool start_flag, bool attack,
                                          unsigned int enemy_keeper);
  std::shared_ptr<agent::halt> halt();
  std::shared_ptr<agent::defense> defense(agent::defense::defense_mode mode,
                                          unsigned int mark_num);
  std::shared_ptr<agent::kick_off> kickoff(bool start_flag);
  std::shared_ptr<agent::kick_off_waiter> kickoff_waiter(
      agent::kick_off_waiter::kickoff_mode mode, bool attack);
  std::shared_ptr<agent::stopgame> stop();
  std::shared_ptr<agent::ball_placement> ball_placement(bool is_ally);
  std::shared_ptr<agent::regular> regular(bool chase_flag);
  std::shared_ptr<agent::setplay> setplay();
  std::shared_ptr<agent::penalty_kick> shootout(bool start_flag, unsigned int enemy_keeper);
};
} // namespace formation
} // namespace game
} // namespace ai_server

#endif
