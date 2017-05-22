#include <unordered_map>
#include <cmath>
#include <tuple>
#include "regular.h"
#include "ai_server/model/robot.h"
#include "ai_server/model/command.h"

namespace ai_server {
namespace game {
namespace agent {

regular::regular(const model::world& world, bool is_yellow,const std::vector<unsigned int>& ids) : base(world, is_yellow), ids_(ids) {
  ball_chase_       = true;
  chase_finished_=false;
  const auto ball   = world_.ball();
  chase_ball_id_ = nearest_robot_id(ball.x(), ball.y(),ids_); //ボール追従ロボ登録
  set_marking_(chase_ball_id_,ball_chase_);
}

bool regular::ball_chase() const {
  return ball_chase_;
}

void regular::set_ball_chase(bool ball_chase) {
  if(ball_chase_!=ball_chase) {
    
    ball_chase_ = ball_chase;  
    
    if(ball_chase_){
      chase_finished_=false;
      kick_finished=false;
      const auto ball   = world_.ball();
      chase_ball_id_ = nearest_robot_id(ball.x(), ball.y(),ids_); //ボール追従ロボ登録
    }

    set_marking_(chase_ball_id_,ball_chase_);    

  }
}

std::vector<std::shared_ptr<action::base>> regular::execute() {
  
  std::vector<std::shared_ptr<action::base>> actions;
  
  
  //ボールを追いかける
  if(ball_chase_){
    if(chase_finished_){
      if(kick_finished){
        
         //ReTry
        const auto ball   = world_.ball();
        unsigned int tmp_chase_ball_id_ = nearest_robot_id(ball.x(), ball.y(), ids_);
        
        if(chase_ball_id_!=tmp_chase_ball_id_){
          std::printf("ReTry!!!!!!!!!!!!!!\n\n");
          chase_ball_id_=tmp_chase_ball_id_;
          chase_finished_=false;
          kick_finished=false;   
          set_marking_(chase_ball_id_,ball_chase_);   
        }
        
      } else{
        std::printf("Kick!\n");
        //Kick
        kick_action_->kick_to(world_.field().x_max(), 0.0);
        kick_action_->set_kick_type(std::tuple<model::command::kick_type_t, double>(model::command::kick_type_t::line, 50.0));
        kick_action_->set_mode(action::kick_action::mode::goal);
        kick_finished= kick_action_->finished();
        actions.push_back(kick_action_);
      }
    } else {
      //Cahse Ball
      chase_finished_=chase_ball_->finished();
      actions.push_back(chase_ball_);
    }
  }

  //marking
  actions.insert(actions.end(),mark_actions_.begin(),mark_actions_.end());
  
  return actions;
}

void regular::set_marking_(unsigned int& chase_ball_id, bool ball_chase){
  const auto those_robots = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  std::vector<unsigned int> this_unadded_ids; //まだ動作の割り当てられていない味方ロボットのID
  std::vector<unsigned int> those_ids; //敵ロボットのID
  unsigned int added_id_ = ids_.at(0); //ある動作を割り当てられる候補のロボットのID
  
  mark_actions_.erase(mark_actions_.begin(),mark_actions_.end());//リセット

  //敵ロボットのIDを登録
  for (auto that_rob : those_robots) {
    those_ids.push_back(that_rob.first);
  }
  
  //マーキング担当のロボットを登録
  if(ball_chase){
    std::printf("\n\x1b[44mChaseBall ID : %d\x1b[49m\n\n",chase_ball_id);
    chase_ball_ = std::make_shared<action::chase_ball>(world_, is_yellow_, chase_ball_id);
   kick_action_ = std::make_shared<action::kick_action>(world_, is_yellow_, chase_ball_id);
    for(auto id: ids_){
      if(id!=chase_ball_id){
        this_unadded_ids.push_back(id);
      }
    }
  }else{
    for(auto id: ids_){
      this_unadded_ids.push_back(id);
    }
  }
  
  
  //対象敵のIDと重要度を登録し、重要度の高い順にソート
  std::vector<regular::that_robot_importance_> that_importance_list;
  for (auto id : those_ids) {
    that_importance_list.push_back({id,-1.0* std::hypot(those_robots.at(id).x() - world_.field().x_min() ,those_robots.at(id).y())});
    //std::printf("\x1b[45m敵ID: %d\x1b[49m  距離： %f mm\n\n",id,std::hypot(those_robots.at(id).x() - world_.field().x_min() ,those_robots.at(id).y()));
  }
  std::sort(that_importance_list.begin(), that_importance_list.end());


  //マーキングを担当するロボットを割り当て
  for (auto that_robot : that_importance_list) {
    if (!this_unadded_ids.empty()) {
      added_id_ = nearest_robot_id(those_robots.at(that_robot.id).x(), those_robots.at(that_robot.id).x(), this_unadded_ids);
      marking_ = std::make_shared<action::marking>(world_, is_yellow_, added_id_);
      marking_->mark_robot(that_robot.id);
      marking_->set_mode(that_robot.importance > -0.0
                             ? action::marking::mark_mode::kick_block
                             : action::marking::mark_mode::shoot_block);
      marking_->set_radius(300.0);
     mark_actions_.push_back(marking_);

      std::printf("\x1b[44m'%d'\x1b[49m marks to \x1b[44m'%d'\x1b[49m\n\n", added_id_,that_robot.id); // debug
    }
     else {
      break;
    }
  }
}

unsigned int regular::nearest_robot_id(double target_x, double target_y,
                                       std::vector<unsigned int>& can_ids) {
  double ret_id         = can_ids.at(0);
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  double robot_target_d;
  double shortest_d = std::hypot(world_.field().x_max(), world_.field().y_max()) + 1000.0;
  auto del_id_itr   = can_ids.begin();

  for (auto id_itr = can_ids.begin(); id_itr < can_ids.end(); ++id_itr) {

    robot_target_d = std::hypot(target_x - ids_robots.at(*id_itr).x(),
                                target_y - ids_robots.at(*id_itr).y());
    if (robot_target_d < shortest_d) {
      // Targetとの距離
        shortest_d = robot_target_d;
        ret_id     = *id_itr;
        del_id_itr = id_itr;
    }
  }
  can_ids.erase(del_id_itr);
  return ret_id;
}

unsigned int regular::nearest_robot_id(double target_x, double target_y,const std::vector<unsigned int>& can_ids){
  double ret_id         = can_ids.at(0);
  const auto ids_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  double robot_target_d;
  double shortest_d = std::hypot(world_.field().x_max(), world_.field().y_max()) + 1000.0;
  auto del_id_itr   = can_ids.begin();

  for (auto id_itr = can_ids.begin(); id_itr < can_ids.end(); ++id_itr) {

    robot_target_d = std::hypot(target_x - ids_robots.at(*id_itr).x(),
                                target_y - ids_robots.at(*id_itr).y());
    if (robot_target_d < shortest_d) {
      // Targetとの距離
        shortest_d = robot_target_d;
        ret_id     = *id_itr;
        del_id_itr = id_itr;
    }
  }
  return ret_id;
}

} // agent
} // game
} // ai_server
