#ifndef AI_SERVER_CONTROLLER_PID_CONTROLLER_H
#define AI_SERVER_CONTROLLER_PID_CONTROLLER_H

#include"base.h"

namespace ai_server{
namespace controller{

class pid_controller:public base{
private:
	double cycle_;
	double kp_;
	double ki_;
	double kd_;

public:
	pid_controller(double cycle) : cycle_(cycle){};
	~pid_controller();

	velocity_t update(const model::robot& robot, const position_t& setpoint);
	velocity_t update(const model::robot& robot, const velocity_t& setpoint);
	void calculate();

};
}	//controller
}	//ai_server

#endif	//AI_SERVER_CONTROLLER_PID_CONTROLLER_H
