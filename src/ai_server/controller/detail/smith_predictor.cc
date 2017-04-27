#include <cmath>

#include "smith_predictor.h"

namespace ai_server {
namespace controller {
namespace detail{

smith_predictor::smith_predictor(const double cycle,const double zeta,const double omega):cycle_(cycle),zeta_(zeta),omega_(omega) {
  for(int i=0;i<7;i++){
    u_[i]={0.0,0.0,0.0};
  }
}

smith_predictor::state smith_predictor::interpolate(const model::robot& robot, const velocity_t u) {
  u_[0]=u;  // 受け取った制御入力を最新入力として

  // 受け取ったロボットの状態(無駄時間分の遅れ含む)
  struct state now_state;
  now_state.p=position_t{robot.x(),robot.y(),robot.theta()};
  now_state.v=velocity_t{robot.vx(),robot.vy(),robot.omega()};
  now_state.a=acceleration_t{robot.ax(),robot.ay(),robot.alpha()};
  
  struct state pre_state=now_state;
  
  // 1ループ毎に当時の制御入力によって1フレーム分の補間をする
  for (int i = 6; i >= 0; i--) {
   now_state.p.x=now_state.p.x+cycle_*pre_state.v.vx;
   now_state.p.y=now_state.p.y+cycle_*pre_state.v.vy;
   now_state.p.theta=now_state.p.theta+cycle_*pre_state.v.omega;
   now_state.v.vx=now_state.v.vx+cycle_*pre_state.a.ax;
   now_state.v.vy=now_state.v.vy+cycle_*pre_state.a.ay;
   now_state.v.omega=now_state.v.omega+cycle_*pre_state.a.alpha;
   now_state.a.ax=now_state.a.ax+cycle_*(-std::pow(omega_,2)*pre_state.v.vx-2*zeta_*omega_*pre_state.a.ax+std::pow(omega_,2)*u_[i].vx);
   now_state.a.ay=now_state.a.ay+cycle_*(-std::pow(omega_,2)*pre_state.v.vy-2*zeta_*omega_*pre_state.a.ay+std::pow(omega_,2)*u_[i].vy);
   now_state.a.alpha=now_state.a.alpha+cycle_*(-std::pow(omega_,2)*pre_state.v.omega-2*zeta_*omega_*pre_state.a.alpha+std::pow(omega_,2)*u_[i].omega);
   
   // 更新
   u_[i]=u_[i-1];
   pre_state=now_state;
  }

  return now_state;
}

} // detail
} // controller
} // ai_serever
