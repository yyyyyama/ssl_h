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
	model::command::position_t first_pos;
	model::command::position_t second_pos;
	model::command::position_t final_pos;
	model::command::position_t my_pos;

	model::command::velocity_t next_vel;

	int phase = 0;
	int select = 0;
	int count = 0;
	int sub_count = 0;

	double target_x = 0;
	double target_y = 0;

	int p1_count = 0;
	int p2_count = 0;

	const double r = 300;
	
	const double acc = 3000;
	const int acc_count = 60;
	const double acc_dist = acc * pow(acc_count / 60., 2) / 2;
	const double max_vel = acc * acc_count / 60.;

	const double rot_omega = 180 * pi<double>() / 180.;
	const double rot_vel = r * rot_omega;
	
	bool fin_flag = false;
};

} // namespace ai_server
} // namespace game
} // namespace action

#endif
