#include <cmath>
#include "move_2walk.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/model/motion/walk_forward.h"
#include "ai_server/model/motion/turn_left.h"
#include "ai_server/model/motion/turn_right.h"
#include "ai_server/model/motion/walk_left.h"
#include "ai_server/model/motion/walk_right.h"
#include "ai_server/model/motion/stop.h"

#include <iostream>         //omega表示のため
using namespace std;        //omega表示のため

namespace ai_server::game::action {
    move_2walk::move_2walk(context& ctx, unsigned int id) : base(ctx, id){}
    bool move_2walk::finished() const{
        return false;
    }

    void move_2walk::set_omega(double omega) {
        omega_ = omega;
    }

           model::command move_2walk::execute() {
        //const double omega = 2.0;

        model::command command{};
        const auto our_robots =model::our_robots(world(), team_color());
    
        if(!our_robots.count(id_)) return command;
            const auto robot = our_robots.at(id_);
            const auto robot_pos =util::math::position(robot);
            const auto ball_pos =util::math::position(world().ball());
    
            //const double omega = util::math::direction_from(std::atan2(ball_pos.y() - robot_pos.y(), ball_pos.x() - robot_pos.x()),robot.theta());
            //std::cout << omega << "\n";      //omega表示のため
            command.set_position(ball_pos, util::math::direction(ball_pos, robot_pos));
 

           // command.set_motion(std::make_shared<model::motion::walk_forward>());
            constexpr double rot_th = 0.5;
            const double pai =3.14;

              if(-rot_th < omega_  &&  rot_th > omega_){
                  command.set_motion(std::make_shared<model::motion::walk_forward>());}

              if(rot_th < omega_ ){
                  command.set_motion(std::make_shared<model::motion::turn_left>());}
              /*  if( omega_ <= 2*pai && omega_ >= pai){
                    command.set_motion(std::make_shared<model::motion::turn_right>());}
                else {command.set_motion(std::make_shared<model::motion::turn_left>());}*/
            

               if(omega_ < -rot_th){
                  command.set_motion(std::make_shared<model::motion::turn_right>());}
                 /*if(omega_ >= -2*pai && omega_ <= -pai){
                    command.set_motion(std::make_shared<model::motion::turn_left>());}
                else{command.set_motion(std::make_shared<model::motion::turn_right>());}*/
            
    
        return command;
    }   

}
