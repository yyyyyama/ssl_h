#ifndef AI_SERVER_GAME_ACTION_CHASE_BALL_H
#define AI_SERVER_GAME_ACTION_CHASE_BALL_H
#include <cmath>
#include "ai_server/model/command.h"
#include "base.h"
#include "ai_server/util/math.h"

namespace ai_server{
namespace game{
namespace action{

using boost::math::constants::pi;

class chase_ball : public base {
public:
	using base::base;
	model::command execute() override;
	bool finished() const override;
	
private:
	model::command::position_t first_pos;			//ボールからrの円周上
	model::command::position_t second_pos;		//最終位置
	model::command::position_t my_pos;				//今の位置

	model::command::velocity_t next_vel;			//次の速度

	int phase = 0;
	int select = 0;
	int count = 0;
	int sub_count = 0;

	double target_x = 0;		//ターゲット
	double target_y = 0;

	int p1_count = 0;		//phase1にかかる時間
	int p2_count = 0;		//phase2にかかる時間

	const double r = 300;			//ボールに対する半径
	
	const double acc = 3000;		//加速度
	const int acc_count = 60;	//加速時間
	const double acc_dist = acc * pow(acc_count / 60., 2) / 2;	//加速にかかる距離
	const double max_vel = acc * acc_count / 60.;	//最高速度

	const double rot_omega = 180 * pi<double>() / 180.;		//円周上を回るときの角速度
	const double rot_vel = r * rot_omega;		//円周上を回るときの速度

	
	bool fin_flag = false;
};

} // namespace ai_server
} // namespace game
} // namespace action

#endif
