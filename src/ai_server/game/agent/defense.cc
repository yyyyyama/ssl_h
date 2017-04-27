#include <cmath>
#include <iostream>

#include "ai_server/game/action/marking.h"
#include "ai_server/game/agent/defense.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace agent {

defense::defense(const model::world& world, bool is_yellow, unsigned int keeper_id,
                 const std::vector<unsigned int>& wall_ids)
    : base(world, is_yellow), keeper_id_(keeper_id), wall_ids_(wall_ids) {}

std::vector<std::shared_ptr<action::base>> defense::execute() {
  using boost::math::constants::pi;

  // actionの生成部分
  //
  // actionの初期化は一回しか行わない
  //
  //
  if (flag_) {
    //キーパー用のaction
    keeper_ = std::make_shared<action::move>(world_, is_yellow_, keeper_id_);

    //壁用のaction
    for (auto it = wall_ids_.begin(); it != wall_ids_.end(); ++it) {
      wall_.push_back(std::make_shared<action::move>(world_, is_yellow_, *it));
			target_x_.push_back(0.0);
			target_y_.push_back(0.0);
    }
  }

  //ボールの座標
  const auto ball_x = world_.ball().x();
  const auto ball_y = world_.ball().y();

  //ゴールの座標
  const auto goal_x = world_.field().x_max();
      

  std::cout << "ball_x:" << ball_x << "ball_y" << ball_y << std::endl;
  std::cout << "goal_x:" << goal_x << std::endl;

  //比
  auto ratio  = 0.0;
  auto length = 0.0;
  //適当な計算用
  auto tmp = 0.0;

  //基準座標を求めている
  //
  //いい感じのところ
  //
  //
  length = std::hypot(goal_x - ball_x, ball_y); //ボール<->ゴール

  ratio = (1340.0) / length; //全体に対しての基準座標の比

  x_ = (1 - ratio) * goal_x + ratio * ball_x;
  y_ = ratio * ball_y;

  if (y_ >= -250 && y_ <= 250) { //もし基準座標が直線の範囲だったら直線に叩き込む
    x_ = (goal_x + (std::signbit(goal_x) ? 1100 : -1100));
  }

  std::cout << "x_:" << x_ << " y_:" << y_ << std::endl;

  //ここから壁の処理
  //
  //
  //
  //
  //

  //基準点からボールへの向き
  const auto theta = util::wrap_to_2pi(std::atan2(ball_y - y_, ball_x - x_));
	
	//壁のイテレータ
  auto wall_it = wall_.begin();

	//壁の目標座標のイテレータ
	auto target_x_it = target_x_.begin();
	auto target_y_it = target_y_.begin();

	//基準点からどれだけずらすか
	auto shift_ = 0.0;

	if(wall_.size()%2 != 0){//奇数
		//(*wall_it)->move_to(x_,y_,theta);
		//wall_it++;
		(*target_x_it) = x_;
		(*target_y_it) = y_;
		target_x_it++;
		target_y_it++;

		shift_ = 180.0;
	}else{//偶数
		shift_ = 90.0;
	}

	//移動した量
	auto move_x = ball_x;
	auto move_y = ball_y;

	//計算の為に中心にずらした場合の座標
	auto after_ball_x = ball_x - move_x;
	auto after_ball_y = ball_y - move_y;


	auto after_base_x = x_ - move_x;
	auto after_base_y = y_ - move_y;


	//x軸から角度
	auto alpha = util::wrap_to_2pi(std::atan2(after_base_y - after_ball_y, after_base_x - after_ball_x));
	
	after_base_x = std::hypot(after_base_x - after_ball_x,after_base_y - after_ball_y);
	after_base_y = 0.0;
	
	//移動した先での仮の座標
	auto tmp_x = 0.0;
	auto tmp_y1 = 0.0;
	auto tmp_y2 = 0.0;
	//回転した後の正しい座標
	auto c_x1 = 0.0;
	auto c_y1 = 0.0;
	auto c_x2 = 0.0;
	auto c_y2 = 0.0;

	for(auto shift = shift_;wall_it!=wall_.end();shift+=shift_,target_x_it++,target_y_it++/*wall_it++*/){
		tmp_x = after_base_x;
		tmp_y1 = std::sqrt(std::pow(shift,2)+std::pow(after_base_x,2)-std::pow(after_base_x,2));
		tmp_y2 = tmp_y1 * (-1);

	/*c_x1 = tmp_x*std::cos(alpha) - tmp_y1*std::sin(alpha) + move_x;
		c_y1 = tmp_x*std::sin(alpha) + tmp_y1*std::cos(alpha) + move_y;
		c_x2 = tmp_x*std::cos(alpha) - tmp_y2*std::sin(alpha) + move_x;
		c_y2 = tmp_x*std::sin(alpha) +	tmp_y2*std::cos(alpha) + move_y;
	
		(*wall_it)->move_to(c_x1,c_y1,theta);
		wall_it++;
		(*wall_it)->move_to(c_x2,c_y2,theta);
	*/
		(*target_x_it) = tmp_x*std::cos(alpha) - tmp_y1*std::sin(alpha) + move_x;
		(*target_y_it) = tmp_x*std::sin(alpha) + tmp_y1*std::cos(alpha) + move_y;
		target_x_it++;
		target_y_it++;
		(*target_x_it) = tmp_x*std::cos(alpha) - tmp_y2*std::sin(alpha) + move_x;
		(*target_y_it) = tmp_x*std::sin(alpha) + tmp_y2*std::cos(alpha) + move_y;
	}

	auto it_y = target_y_.begin();
	auto my_robot = 
	for(auto it_x = target_x_.begin();it_x!=target_x_.end();it_x++,it_y++;){
		auto slope = ((*it_y)-現在地)/((*it_x)-現在地);
		auto segment = it_y - slope * it_x;
		auto D = std::pow(segment,2)-4*(1+std::pow(slope,2))*(std::pow(segment,2)-std::pow(1250,2)-std::pow(goal_x,2));
	}

	//ここからキーパーの処理
  //
  //キーパーはボールの位置によって動き方が3種類ある.
  //
  // A:ボールが敵陣地なので多分そこまで動く必要はない
  // B:ボールが自陣地なので壁の補強をしなければ
  // C:ボールはゴールの直ぐ目の前なのでゴールまえでジャンプしてでも止める
  //
  //
  const auto demarcation = 2250.0; //縄張りの大きさ

	auto keeper_y = 0.0;
	auto keeper_x = 0.0;
	auto keeper_theta = 0.0;

  if (std::signbit(ball_x) == std::signbit(goal_x * (-1))) { // A
    //ゴール直前でボールに併せて横移動

    keeper_x = goal_x + (std::signbit(world_.field().x_max()) ? 100.0 : -100.0);
		keeper_y = ((ball_y >= -500 && ball_y <= 500) ? ball_y : 0);

  } else if (std::signbit(std::pow(ball_x - world_.field().x_max(), 2) + std::pow(ball_y, 2) -
                          std::pow(demarcation, 2))) { // C
    //ゴール前でディフェンスする

    //ゴール前で張ってるキーパの位置
    length = std::hypot(goal_x - ball_x, ball_y); //ゴール<->ボール
    ratio  = (400) / length; //全体に対してのキーパー位置の比
		
		keeper_x = ((1 - ratio) * goal_x + ratio * ball_x);	
		keeper_y = ratio * ball_y;

    //もし支線の先にロボットがいたら
    //そのロボットの視線の先に移動
    //セット仕直し
  } else { // B
    //壁のすぐ後ろで待機

    //基準点からちょっと下がったキーパの位置
    length = std::hypot(goal_x - ball_x, ball_y); //基準点<->ボール
    ratio  = (800) / length; //全体に対してのキーパー位置の比
		
		keeper_x = (1 - ratio) * goal_x + ratio * ball_x;
		keeper_y = ratio * ball_y;
  }
	keeper_theta = util::wrap_to_2pi(std::atan2(ball_y-keeper_y, ball_x-keeper_x));

	keeper_->move_to(keeper_x,keeper_y,keeper_theta);//置く場所をセット


	if (flag_ != true) { //もし一回目のループではなかったら前回のキーパーを削除
    wall_.pop_back();
  }
  flag_ = false; //一回でも呼ばれたらfalseにする

  wall_.push_back(keeper_); //配列を返すためにキーパーを統合する
  std::vector<std::shared_ptr<action::base>> re_wall{
      wall_.begin(), wall_.end()}; //型を合わせるために無理矢理作り直す

  return re_wall; //返す
}
}
}
}
