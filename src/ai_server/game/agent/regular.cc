#include <unordered_map>

#include "regular.h"
#include "ai_server/model/robot.h"

namespace ai_server {
namespace game {
namespace agent {

regular::regular(const model::world& world, bool is_yellow,
                 const std::vector<unsigned int>& ids)
    : base(world, is_yellow), ids_(ids) {}

std::vector<std::shared_ptr<action::base>> regular::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
      const auto ball            = world_.ball();
      const auto this_robot_team = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
      const auto that_robot_team = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
      std::vector<model::robot> that_robots;
      std::vector<unsigned int> added_ids; //動作が既に割当てられているロボットのID
      unsigned int candidate_id;  //ある動作を割り当てられる候補のロボットのID
      
      //味方・敵ロボットの登録
      for(auto id : ids_) {
		 this_robots.push_back(this_robot_team.at(id));
	  }
	  for(auto that_rob : that_robot_team){
		  that_robots.push_back(that_rob.second);
	  }
	  
	  //ボールを追いかけるロボットの登録
      double shortest_d=this_robots.at(0);
	    for(auto this_robot = this_robots.begin(); this_robot != this_robots.end(); ++this_robot) {
				  //ボールとの距離
                  
			
			
        break;
    }

	  
	  //ボールに対する角度
	  //似たり寄ったりの時は、登録順が若いのを設定
	  
	  
	  //マーキングするロボットの登録
  
  return actions;
}

} // agent
} // game
} // ai_server
