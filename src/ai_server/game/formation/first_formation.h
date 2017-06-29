#ifndef AI_SERVER_GAME_FORMATIOIN_FIRST_FORMATION_H
#define AI_SERVER_GAME_FORMATIOIN_FIRST_FORMATION_H

#include <memory>
#include <vector>
#include <boost/variant.hpp>

#include "base.h"
#include "ai_server/model/refbox.h"
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
    setplay_defense
  };

  first_formation(const model::world& world, const model::refbox& refcommand, bool is_yellow,
                  const std::vector<unsigned int>& ids);

  std::vector<std::shared_ptr<agent::base>> execute() override;

private:
  void decide_keeper();
  void decide_wall_number(); //壁の数を決定
  void decide_wall();        //壁にするロボットを決定
  void decide_kicker(); //壁,キーパー以外でボールに一番近いロボットをキッカーに決定
  void other_robots();  //キーパー,壁以外のロボットを抽出
  void waiter();        //キーパー,壁,キッカー以外のロボットを抽出
  void except_keeper(); //キーパー以外のロボットを抽出

  //コマンドを使いやすい形に変形
  command convert_command(model::refbox::game_command command);
  //ボールを蹴ったか判定
  bool kicked();

  //役割に対するid
  int wall_number_;                         //壁の数
  unsigned int keeper_;                     //キーパー
  std::vector<unsigned int> wall_;          //壁
  unsigned int kicker_;                     //キッカー
  std::vector<unsigned int> others_;        //キーパー,壁以外
  std::vector<unsigned int> waiter_;        //キーパー,壁,キッカー以外
  std::vector<unsigned int> except_keeper_; //キーパー以外

  std::vector<unsigned int> ids_;

  bool kicked_flag_; //ボールが蹴られたか判断するためのフラグ
  bool regular_flag_; //定常状態に遷移した際に、一回だけagentを初期化したいため必要
  void reset_flag(); // kicked_flag_,regular_flag_を初期化

  model::refbox::game_command previous_refcommand_;
  command previous_command_;
  command current_command_;
  model::ball previous_ball_;

  std::vector<std::shared_ptr<agent::base>> exe_;
  std::shared_ptr<agent::penalty_kick> pk_;
  std::shared_ptr<agent::halt> halt_;
  std::shared_ptr<agent::defense> df_;
  std::shared_ptr<agent::kick_off> kickoff_;
  std::shared_ptr<agent::kick_off_waiter> kickoff_waiter_;
  std::shared_ptr<agent::stopgame> stop_;
  std::shared_ptr<agent::regular> regular_;
  std::shared_ptr<agent::setplay> setplay_;

  // agentを呼び出す関数
  void pk(bool start_flag, bool attack);
  void halt();
  void defense(agent::defense::defense_mode mode, bool mark_flag);
  void kickoff(bool start_flag);
  void kickoff_waiter(agent::kick_off_waiter::kickoff_mode mode, bool attack);
  void stop();
  void regular(bool chase_flag);
  void setplay();
};
}
}
}

#endif
