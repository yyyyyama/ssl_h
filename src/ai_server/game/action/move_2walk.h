#ifndef AI_SERVER_GAME_ACTION_MOVE_2WALK_H
#define AI_SERVER_GAME_ACTION_MOVE_2WALK_H

#include <Eigen/Core>
#include "base.h"

namespace ai_server::game::action{

class move_2walk : public base {
  public:
    move_2walk(context& ctx, unsigned int id);
     
    /// @brief 実行
    /// @return ロボットに送信するコマンド
    model::command execute() override;
    
   /* /// @brief 2足歩行移動 
    void move_2walk(double omega,double rot_th,double pai);
  */
    /// @return 動作が完了しているか?
    bool finished() const override;

    void set_omega(double omega);

  private:    
    double omega_;
};

}
#endif