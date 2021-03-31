#ifndef AI_SERVER_RADIO_BASE_BASE_H
#define AI_SERVER_RADIO_BASE_BASE_H

#include "ai_server/model/command.h"
#include "ai_server/model/team_color.h"

namespace ai_server::radio::base {

/// ロボットへの命令の送信
class command {
public:
  virtual ~command() = default;

  /// @brief            ロボットへ命令を送信する
  /// @param color      チームカラー
  /// @param id         ロボットの ID
  /// @param kick_flag  キッカーへの命令
  /// @param dribble    ドリブラーへの命令
  /// @param vx         x 軸方向の速度
  /// @param vy         y 軸方向の速度
  /// @param omega      角速度
  virtual void send(model::team_color color, unsigned int id,
                    const model::command::kick_flag_t& kick_flag, int dribble, double vx,
                    double vy, double omega) = 0;

  virtual void send(model::team_color color, unsigned int id,
                    std::shared_ptr<model::motion::base> motion) = 0;
};

/// シミュレータの制御コマンドの送信
class simulator {
public:
  virtual ~simulator() = default;

  /// @brief        シミュレータ上のボールを指定した座標に配置する
  /// @param x      x 座標 [mm]
  /// @param y      y 座標 [mm]
  virtual void set_ball_position(double x, double y) = 0;

  /// @brief        シミュレータ上のロボットを指定した座標に配置する
  /// @param color  チームカラー
  /// @param id     ロボットの ID
  /// @param x      x 座標 [mm]
  /// @param y      y 座標 [mm]
  /// @param theta  方向 [rad]
  virtual void set_robot_position(model::team_color color, unsigned int id, double x, double y,
                                  double theta) = 0;
};

} // namespace ai_server::radio::base

#endif // AI_SERVER_RADIO_BASE_BASE_H
