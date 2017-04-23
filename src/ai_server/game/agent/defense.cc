#include <cmath>
#include <iostream>

#include "ai_server/game/agent/defense.h"
#include "ai_server/game/action/move.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace agent {

defense::defense(const model::world& world, bool is_yellow, unsigned int keeper_id,
                 const std::vector<unsigned int>& wall_ids)
    : base(world, is_yellow), keeper_id_(keeper_id), wall_ids_(wall_ids) {}

std::vector<std::shared_ptr<action::base>> defense::execute() {
  using boost::math::constants::pi;

  //キーパー用のaction
  std::shared_ptr<action::move> keeper =
      std::make_shared<action::move>(world_, is_yellow_, keeper_id_);
	
  //壁用のaction
  std::vector<std::shared_ptr<action::move>> wall;
  for (auto it = wall_ids_.begin(); it != wall_ids_.end(); ++it) {
    wall.push_back(std::make_shared<action::move>(world_, is_yellow_, *it));
  }


  //ボールの座標
  const auto ball_x = world_.ball().x();
  const auto ball_y = world_.ball().y();
  //ゴールの座標
  const auto goal_x =world_.field().x_max();
	
	std::cout << "ball_x:" << ball_x << "ball_y" << ball_y << std::endl;
	
  //比
  auto ratio  = 0.0;
  auto length = 0.0;
  //適当な計算用
  auto tmp = 0.0;

  //各ロボットの基準点.今回はボールとゴールの直線とボールエリアの境界線の交点
  length = std::hypot(goal_x - ball_x, ball_y); //ボールとゴールの距離

  ratio = (1500.0) / length; //全体に対してのゴールエリアの大きさの比
                             //基準座標
  x_ = (1 - ratio) * goal_x + ratio * ball_x;
  y_ = ratio * ball_y;
	std::cout << "x_:" << x_ << " y_:" << y_ << std::endl;
  //基準点とボールを結んだ直線と垂直に交わる直線の傾き
  const auto inclination = -((ball_y - y_) / (ball_x - x_));
  //基準点とボールを結んだ直線と垂直に交わる直線の切片
  const auto segment = y_ - inclination * x_;
  //基準点からボールへの向き
  const auto theta = util::wrap_to_2pi(std::atan2(ball_y - y_, ball_x - x_) + pi<double>());

  //ここから壁の処理
  //
  //壁のイテレータ
  auto wall_it = wall.begin();
  //基準点からどれだけずらすか
  auto shift = 0.0;

  //壁の数が偶数奇数の判定
  if (wall_ids_.size() % 2 != 0) { //奇数
		(*wall_it)->move_to(x_, y_, theta);
    ++wall_it;
    shift = 180.0;
  } else {
    shift = 90.0;
  }
	
  //直線を引くための適当な点
  const auto tmp_x = 1000.0;
  const auto tmp_y = inclination * tmp_x + segment;

  //基準点から左右に配置するロボットの座標
  length = std::hypot(x_ - tmp_x, y_ - tmp_y); //基準点<->適当な点の長さ

  // shift_real : 実際に足したり引いたりされるずらし具合
  for (auto shift_real = shift; wall_it != wall.end(); shift_real *= -1, ++wall_it) {
    if (std::signbit(shift_real)) { //基準点より左側
      tmp   = length / (length + shift_real);
      ratio = 1 - tmp;
      (*wall_it)->move_to((-ratio * tmp_x + x_) / tmp, (-ratio * tmp_y + y_) / tmp, theta);
      shift_real -= shift;
    } else {                         //基準点より右側
      ratio = (shift_real) / length; //全体に対してのずらし具合の比
      (*wall_it)->move_to((1 - ratio) * x_ + ratio * tmp_x, (1 - ratio) * y_ + ratio * tmp_y,
                          theta); //置く場所をセット
    }
  }

	
  
	//ここからキーパーの処理
  //縄張りの大きさ
  const auto demarcation = 2250.0;

  if (std::signbit(ball_x) ==
      std::signbit(goal_x * (-1))) { //ボールは敵陣地なのでキーパーはさがる:A
    //ゴール直前でボールに併せて横移動

    keeper->move_to(goal_x, ball_y,
                    util::wrap_to_2pi(std::atan2(0, goal_x - ball_x) + pi<double>()));

  } else if (std::signbit(
                 std::pow(ball_x - world_.field().x_max(), 2) + std::pow(ball_y, 2) -
                 std::pow(demarcation, 2))) { //ボールは縄張りのなかなので頑張れキーパー:C
    //ゴール前で張ってるキーパの位置
    length = std::hypot(x_ - ball_x, y_ - ball_y); //基準点<->ボール
    ratio  = (180) / length; //全体に対してのキーパー位置の比

    keeper->move_to((1 - ratio) * x_ + ratio * tmp_x, (1 - ratio) * y_ + ratio * tmp_y,
                    theta); //置く場所をセット

    //もし支線の先にロボットがいたら
    //そのロボットの視線の先に移動
    //セット仕直し
  } else { //ボールはこっち陣地なので壁の補強:B
    //壁のすぐ後ろで待機

    //基準点からちょっと下がったキーパの位置
    length = std::hypot(x_ - ball_x, y_ - ball_y); //基準点<->ボール
    ratio  = (800) / length; //全体に対してのキーパー位置の比

    keeper->move_to((1 - ratio) * x_ + ratio * tmp_x, (1 - ratio) * y_ + ratio * tmp_y,
                    theta); //置く場所をセット
  }
	
  //wall.push_back(keeper); //配列を返すためにキーパーも統合する
	std::vector<std::shared_ptr<action::base>> re_wall{wall.begin(),wall.end()};
  return re_wall; //返す
}
}
}
}
