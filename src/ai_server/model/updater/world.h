#ifndef AI_SERVER_MODEL_WORLD_UPDATER_WORLD_H
#define AI_SERVER_MODEL_WORLD_UPDATER_WORLD_H

#include "ai_server/model/world.h"
#include "ball.h"
#include "field.h"
#include "robot.h"

// 前方宣言
namespace ssl_protos {
namespace vision {
class Packet;
}
}

namespace ai_server {
namespace model {
namespace updater {

class world {
  /// フィールドのupdater
  field field_;
  /// ボールのupdater
  ball ball_;
  /// 青ロボットのupdater
  robot<model::team_color::blue> robots_blue_;
  /// 黄ロボットのupdater
  robot<model::team_color::yellow> robots_yellow_;

public:
  world()             = default;
  world(const world&) = delete;
  world& operator=(const world&) = delete;

  /// @brief                  内部の状態を更新する
  /// @param packet           SSL-Visionのパース済みパケット
  void update(const ssl_protos::vision::Packet& packet);

  /// @brief           値を取得する
  model::world value() const;

  /// @brief           フィールドのupdaterを取得する
  field& field_updater();
  /// @brief           ボールのupdaterを取得する
  ball& ball_updater();
  /// @brief           青ロボットのupdaterを取得する
  robot<model::team_color::blue>& robots_blue_updater();
  /// @brief           黄ロボットのupdaterを取得する
  robot<model::team_color::yellow>& robots_yellow_updater();
};

} // namespace updater
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_WORLD_UPDATER_BALL_H
