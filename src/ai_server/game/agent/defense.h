#ifndef AI_SERVER_GAME_ACTION_DEFENSE_H
#define AI_SERVER_GAME_ACTION_DEFENSE_H

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "ai_server/game/action/guard.h"
#include "ai_server/game/action/get_ball.h"
#include "ai_server/game/action/marking.h"
#include "ai_server/game/agent/base.h"

namespace ai_server {
namespace game {
namespace agent {

class defense : public base {
public:
  struct enemy {
    //ロボットのid
    unsigned int id;
    //ロボットの位置
    Eigen::Vector2d position;
    //ロボットの角度
    double theta;
    //ソート計算した値を一時的に格納する変数
    double valuation;
    //評価点数
    unsigned int score;
  };
  struct mark_tmp {
    //ロボットの位置
    Eigen::Vector2d position;
    //マーキングのアクション
    unsigned int id;
  };
  struct mark_g {
    //ロボットの位置
    Eigen::Vector2d position;
    //マーキングのアクション
    std::shared_ptr<action::guard> action;
  };
  struct mark {
    //ロボットの位置
    Eigen::Vector2d position;
    //マーキングのアクション
    std::shared_ptr<action::marking> action;
  };
  defense(const model::world& world, bool is_yellow, unsigned int keeper_id,
          const std::vector<unsigned int>& wall_ids,
          const std::vector<unsigned int>& marking_ids);
  defense(const model::world& world, bool is_yellow, unsigned int keeper_id,
          const std::vector<unsigned int>& wall_ids);
  enum class defense_mode { normal_mode, pk_normal_mode, pk_extention_mode, stop_mode };
  void set_mode(agent::defense::defense_mode mode);
  std::vector<unsigned int> marking() const;
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  unsigned int keeper_id_;
  const std::vector<unsigned int> wall_ids_;
  const std::vector<unsigned int> marking_ids_;
  std::vector<unsigned int> marking_ids_re_;
  std::shared_ptr<action::guard> keeper_;
  std::shared_ptr<action::get_ball> keeper_g_;
  std::vector<std::shared_ptr<action::guard>> wall_;
  std::vector<std::shared_ptr<action::get_ball>> wall_g_;
  std::vector<std::shared_ptr<action::marking>> marking_;
  std::vector<std::shared_ptr<action::guard>> marking_g_;
  Eigen::Vector2d orientation_;
  defense_mode mode_;
  Eigen::Vector2d ball_;
  bool is_er_;
};
}
}
}
#endif // AI_SERVER_GAME_ACTION_DEFENSE_H
