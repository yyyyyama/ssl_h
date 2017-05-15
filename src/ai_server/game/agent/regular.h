#ifndef AI_SERVER_GAME_AGENT_REGULAR_H
#define AI_SERVER_GAME_AGENT_REGULAR_H

#include <unordered_map>
#include <tuple>
#include <cmath>
#include <utility>
#include <boost/geometry/geometry.hpp>

#include "base.h"
#include "ai_server/game/action/marking.h"
#include "ai_server/game/action/chase_ball.h"
#include "ai_server/game/action/kick_action.h"
#include "ai_server/model/robot.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace agent {

class regular : public base {
public:
  regular(const model::world& world, bool is_yellow, const std::vector<unsigned int>& ids);
  void set_ball_chase(bool ball_chase); //全てのロボットをマーキングに割り当てる時はfalse
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  const std::vector<unsigned int>& ids_;
  bool ball_chase_;

  bool chase_ball_finished_;
  bool kick_action_finished_;

  // Action
  std::shared_ptr<action::marking> marking_;
  std::shared_ptr<action::chase_ball> chase_ball_;
  std::shared_ptr<action::kick_action> kick_action_;

  // IDと、優先度の構造体
  //重要度の所には、「(条件1)*a+(条件2)*b+...
  //(a,b,...は任意の数)」を代入し、重要度の高い順にロボットが対応する。a,bには条件の優先度合いを指定する。
  struct id_importance_ {
    unsigned int id;   // ID
    double importance; //重要度
    //ソート基準を重要度の高い順に設定
    bool operator<(const id_importance_& next) const {
      return importance > next.importance;
    }
  };

  //ゾーンの構造体(四角形)
  struct zone_ {
    unsigned int id; //ゾーンのID
    double x_min;
    double y_min;
    double x_max;
    double y_max;
  };

  std::vector<zone_> zones_; //ゾーンのvector

  //ロボットとゾーンの結び付け
  using id_id_ = std::unordered_map<unsigned int, unsigned int>;
  id_id_ robot_id_zone_id_;
  id_id_ zone_id_robot_id_;

  unsigned int chase_ball_id;

  void set_chase_ball_id_(); //ボールを追いかけるロボットを設定

  void set_zones_(); //ゾーンの設定

  void set_robot_ids_zone_ids_(); //ロボットIDとゾーンID関係性を設定

  //ある点が指定されたエリア上にあるか(第2引数のzone_構造体のIDは適当で良い)
  bool on_area_(double x, double y, zone_ zone);

  //ある地点に対応するゾーンIDを返す
  unsigned int point_to_zone_id_(double x, double y);
  
  //安定マッチング
  std::vector<unsigned int> stable_matching_(
      const std::vector<std::vector<unsigned int>>& order_man,
      const std::vector<std::vector<unsigned int>>& order_woman);
};

} // agent
} // game
} // ai_server

#endif
