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
						wall.push_back(std::make_shared<action::base>(*it));
					}
					
					//ボールの座標
					const auto ball_x = world_.ball().x();
					const auto ball_y = world_.ball().y();
					//ゴールの座標
					const auto goal_x = -1 * world_.field().x_max();

					//比
					auto ratio = 0.0;
					auto length = 0.0;
					//適当な計算用
					auto tmp = 0.0;

					//各ロボットの基準点.今回はボールとゴールの直線とボールエリアの境界線の交点
					length = std::hypot(goal_x - ball_x, ball_y); //ボールとゴールの距離

      		ratio = (1500.0) / length;//全体に対してのゴールエリアの大きさの比
					//基準座標
      		x_     = (1 - ratio) * goal_x + ratio * ball_x;
      		y_     = ratio * ball_y;
				
					//基準点とボールを結んだ直線と垂直に交わる直線の傾き
					const auto inclination = -((ball_y - y_)/(ball_x - x_));
					//基準点とボールを結んだ直線と垂直に交わる直線の切片
					const auto segment = y_ - inclination*x_;
					//基準点からボールへの向き
					const auto theta = util::wrap_to_2pi(std::atan2(y - tmp_y, x - tmp_x) + pi<double>());

					//壁のイテレータ
					auto wall_it = wall.begin();
					//基準点からどれだけずらすか
					auto shift = 0.0;

					//壁の数が偶数奇数の判定
					if(ids_.size()%2 == 0){//偶数
						(*wall_it)->move_to(x_,y_,theta);
						++wall_it;
						shift = 180.0;
					}else{
						shift = 90.0;
					}

					//直線を引くための適当な点
					const auto tmp_x = 1000.0;
					const auto tmp_y = inclination*tmp_x + segment;
					
					//基準点から左右に配置するロボットの座標
					length = std::hypot(x_ - tmp_x, y_ - tmp_y); //基準点<->適当な点の長さ

					//shift_real : 実際に足したり引いたりされるずらし具合
					for(auto shift_real = shift; wall_it != wall.end(); shift_real*-1 , ++wall_it){
						if(shift_real<0){
							tmp = length / (length + shift_real); //敵位置 - 自位置の比
      				ratio = 1 - tmp;
							(*wall_it->move_to((-ratio * tmp_x + x_) / tmp , (-ratio * tmp_y + y_) / tmp ,theta);
							shift_real-=shift;
						}else{
							ratio = (shift_real) / length;//全体に対してのずらし具合の比
      				(*wall_it)->move_to( (1 - ratio) * x_ + ratio * tmp_x , (1 - ratio) * y_ + ratio * tmp_y , theta );//置く場所をセット
						}
					}
      		
			}

		}
	}
}
