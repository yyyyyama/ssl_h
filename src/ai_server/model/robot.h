#include<iostream>

#ifndef INCLUDED_ROBOT_H
#define INCLUDED_ROBOT_H

namespace ai_server{
namespace model{
	
	class robot{
		
	private:
		int id_;
		double x_;
		double y_;
		double vx_;
		double vy_;
		double theta_;
		double omega_;
		
		
	public:
		robot(unsigned int id);
		robot(unsigned int id, double x, double y);
		
		int id();
		double x();
		double y();
		double vx();
		double vy();
		double theta();
		double omega();
		
		void set_x(double x);
		void set_y(double y);
		void set_vx(double vx);
		void set_vy(double vy);
		void set_theta(double theta);
		void set_omega(double omega);
		
	};
	
}
}


#endif
