#include <Eigen/Core>
#include "ai_server/game/action/chase_ball.h"

namespace ai_server {
namespace game {
namespace action {

model::command chase_ball::execute() {
	model::command command(id_);
	const auto fri_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
	const auto& robot = fri_robots.at(id_);

	using namespace Eigen;

	const auto ball_x = world_.ball().x();
	const auto ball_y = world_.ball().y();

	const auto ball_vx = world_.ball().vx();
	const auto ball_vy = world_.ball().vy();   

	my_pos = {robot.x(), robot.y(), robot.theta()};

	Vector2d v1(ball_x - my_pos.x, ball_y - my_pos.y);
	v1.normalized();

	second_pos = {ball_x - r * v1(0), ball_y - r * v1(1), std::atan2(v1(0),v1(1))};

	Vector2d v2(target_x - ball_x, target_y - ball_y); 
	v2.normalized();
	
	final_pos = {ball_x - r * v2(0), ball_y - r * v2(1), std::atan2(v2(0),v2(1))};

	auto dist1 = std::hypot(my_pos.x - second_pos.x, my_pos.y - second_pos.y);
	auto dist2 = std::hypot(my_pos.x - final_pos.x, my_pos.y - final_pos.y);

	auto move_angle = util::wrap_to_pi(v2(2) - v1(2));

	if (phase == 0) {
		target_x = 4500.;
		target_y = 0.;

		if (dist1 >= acc_dist * 2) {
			p1_count = acc_count * 2 + (dist1 - acc_dist * 2) / max_vel;
			select = 1;
		}
		else {
			p1_count = std::sqrt(2 * (dist1 / 2.) / acc) * 60 * 2;
			select = 2;
		}

		p2_count = std::abs(move_angle) / (rot_omega / 60.);

		phase = 1;
	}

	if (phase == 1) {
		count++;
		auto omega1 = ( util::wrap_to_pi(second_pos.theta - my_pos.theta) ) / (p1_count - count);

		switch (select) {
			case 1 : if (count <= acc_count) {
									next_vel = {acc * count / 60. * std::cos(v1(2)), acc * count / 60. * std::sin(v1(2)), omega1};
								}
								else if (count > acc_count && dist1 >= acc_dist) {
									next_vel = {max_vel * std::cos(v1(2)), max_vel * std::sin(v1(2)), omega1};
								}
								else if (dist1 < acc_dist) {
									sub_count++;
									if(sub_count <= acc_count) {
										next_vel = {acc * (acc_count - sub_count) / 60. * std::cos(v1(2)), acc * (acc_count - sub_count) / 60. * std::sin(v1(2)), omega1};
									}
									if(dist1 < 100) {
										phase = 2;
									}
								}
								break;

			case 2 : if (count <= p1_count / 2) {
									next_vel = {acc * count / 60. * std::cos(v1(2)), acc * count / 60. * std::sin(v1(2)), omega1};
								}
								else {
									sub_count++;
									if (sub_count < p1_count) {
										next_vel = {acc * (p1_count / 2. - sub_count) / 60. * std::cos(v1(2)), acc * (p1_count / 2. - sub_count) / 60. * std::sin(v1(2)), omega1};
									}
									if(dist1 < 100) {
										count = 0;
										sub_count = 0;
										phase = 2;
									}
								}
								break;
		}
	} 

	else if (phase == 2){
		count++;
		if (count <= p2_count) {
			if (move_angle >= 0) {
				next_vel = {rot_vel * std::sin(v1(2)), -rot_vel * std::cos(v1(2)), rot_omega / 60.};
			}
			else {
				next_vel = {-rot_vel * std::sin(v1(2)), rot_vel * std::cos(v1(2)), -rot_omega / 60.};
			}
		}

		else if (dist2 > 30 || std::abs(move_angle) > 1) {
			next_vel = {0,0,0};
		}
		else {
			next_vel = {0,0,0};
			phase = 3;
		}
	}
	
	else if (phase == 3) {
		next_vel = {300 * std::cos(v1(2)), 300 * std::sin(v1(2)), 0};
		command.set_dribble(5);
		fin_flag = true;
	}

	next_vel = {next_vel.vx + ball_vx, next_vel.vy + ball_vy, next_vel.omega};
	command.set_velocity(next_vel);

	return command;
}

bool chase_ball::finished() const {
	return fin_flag;
}

} // namespace ai_server
} // namespace game
} // namespace action
