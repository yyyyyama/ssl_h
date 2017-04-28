#include "penalty_kick.h"
#include "ai_server/model/command.h"
#include "ai_server/util/math.h"

#include <cmath>
#include <boost/math/constants/constants.hpp>

namespace ai_server{
namespace game{
namespace agent{

penalty_kick::penalty_kick(const model::world& world, bool is_yellow, unsigned int kicker_id, const std::vector<unsigned int>& ids) 
                         : base(world, is_yellow){
                               kicker_id_ = kicker_id;
                               move = std::make_shared<action::move>(world, is_yellow, kicker_id);
                               kick = std::make_shared<action::kick_action>(world, is_yellow, kicker_id);
                               start_flag_ = false;
                               kick_x = 0;
                               kick_y = 0;
                               kick_theta = 0;
                               target_x = 0;
                               target_y = 0;
}

bool penalty_kick::start_flag() const {
    return start_flag_;
}

void penalty_kick::set_start_flag(bool start_flag){
    start_flag_ = start_flag;
}

std::vector<std::shared_ptr<action::base>> penalty_kick::execute(){ 
    calculate();
    if(!exe.empty()){
        exe.pop_back();
    }
    if(!start_flag_){
        if(!move->finished()){
            //移動
            move->move_to(kick_x, kick_y, kick_theta);
            exe.push_back(move);
        }else{
            //何もしない
            auto nop = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
            start_flag_ = true;
            exe.push_back(nop);
        }
    }else{
        if(!kick->finished()){
            kick->kick_to(target_x, target_y);
            model::command::kick_flag_t kick_flag(model::command::kick_type_t::line, 30.0);
            kick->set_kick_type(kick_flag);
            exe.push_back(kick);          
        }else{
            //何もしない
            auto nop = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
            exe.push_back(nop);
            
        }
    }
    return exe;
}

void penalty_kick::calculate(){
    using namespace boost::math::constants;
    const auto goal_x = world_.field().x_min();
    const auto goal_y = 0;
    const auto ball_x = world_.ball().x();
    const auto ball_y = world_.ball().y();

    target_x = goal_x;
    target_y = goal_y + 300;

    //PK待機位置
    //パラメータ計算
    double b = 1; double a = (target_y - ball_y) / (target_x - ball_x); double c = -ball_y + (-a * ball_x);
    double l = a * a + b * b;
    double k = a * ball_x + b * ball_y + c;
    double d = l * 500 * 500 - k * k;
    //位置計算
    if (d > 0){
        kick_x = (ball_x - a / l * k) + (b / l * sqrt(d));
    }else if (d == 0){
        kick_x = ball_x - a * k / l;
        
    }else{
        kick_x = ball_x + 500;
    }
    kick_y = (ball_y < 0) ? -sqrt((500 * 500) - (kick_x - ball_x) * (kick_x - ball_x)) + ball_y : (sqrt((500 * 500) - (kick_x - ball_x) * (kick_x - ball_x)) + ball_y);
    kick_theta = atan2((target_y - ball_y), (target_x - ball_x));

}

}
}
}