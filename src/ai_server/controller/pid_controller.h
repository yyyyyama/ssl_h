#ifndef AI_SERVER_CONTROLLER_PID_CONTROLLER_H
#define AI_SERVER_CONTROLLER_PID_CONTROLLER_H

#include "base.h"

namespace ai_server {
namespace controller {

class pid_controller : public base {
private:
  double cycle_;            //制御周期
  const static double kp_;  //比例ゲイン
  const static double ki_;  //積分ゲイン
  const static double kd_;  //微分ゲイン
  velocity_t up_[2];        //操作量(比例,1フレーム前まで)
  velocity_t ui_[2];        //操作量(積分,1フレーム前まで)
  velocity_t ud_[2];        //操作量(微分,1フレーム前まで)
  velocity_t u_;            //操作量
  velocity_t e_[2];         //偏差(1フレーム前まで)
  double max_velocity_;     //最大速度
  double max_acceleration_; //最大加速度

  //制御計算関数
  void calculate();

public:
  //コンストラクタ
  pid_controller(double cycle);

  //制御入力更新関数
  velocity_t update(const model::robot& robot, const position_t& setpoint);
  velocity_t update(const model::robot& robot, const velocity_t& setpoint);
};

} // controller

//以下，model::command::velocityの四則演算のオーバロード
const model::command::velocity_t operator+(const model::command::velocity_t& vel1,
                                           const model::command::velocity_t& vel2);
const model::command::velocity_t operator-(const model::command::velocity_t& vel1,
                                           const model::command::velocity_t& vel2);
const model::command::velocity_t operator*(const double& c,
                                           const model::command::velocity_t& vel);
const model::command::velocity_t operator/(const model::command::velocity_t& vel,
                                           const double& c);
} // ai_server

#endif // AI_SERVER_CONTROLLER_PID_CONTROLLER_H
