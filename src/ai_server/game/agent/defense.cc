#include "ai_server/game/agent/defense.h"

namespace ai_server{
	namespace game{
		namespace action{
			
			defense::defense(const model::world& world,bool is_yellow,unsigned int keeper_id,const std::vector<unsigned int>& ids):keeper_id_(keeper_id),ids_(ids){}

			std::vector<std::shared_ptr<action::base>> defens::execute(){

			  	//キーパー用のaction
				  action::move keeper(keeper_id_);
					
				  //壁用のaction
					std::vector<std::shared_ptr<action::base>> wall;
					for(auto it = ids_.begin() ; it!=ids_.end() ; ++it){
						wall.push_back((std::make_shared<action::base>(*it));
					}
					
					//ボールの座標
					const auto ball_x = world_.ball().x();
					const auto ball_y = world_.ball().y();
					//ゴールの座標
					const auto goal_x = -1 * world_.field().x_max();

					//各ロボットの基準点.今回はボールとゴールの直線とボールエリアの境界線の交点
					const auto length = std::hypot(goal_x - ball_x, ball_y); //ボールとゴールの距離
      		ratio = (1500.0) / length;//全体に対してのゴールエリアの大きさの比
					//基準座標
      		x_     = (1 - ratio) * goal_x + ratio * ball_x;
      		y_     = ratio * ball_y;
					
			}

		}
	}
}
#endif//AI_SERVER_GAME_ACTION_DEFENSE_H
