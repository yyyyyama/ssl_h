#include <cmath>

#include "ai_server/game/agent/defense.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace agent {

defense::defense(const model::world& world, bool is_yellow, unsigned int keeper_id,
                 const std::vector<unsigned int>& wall_ids)
    : base(world, is_yellow), keeper_id_(keeper_id), wall_ids_(wall_ids), x_(0.0), y_(0.0) {
  // actionの生成部分
  //
  // actionの初期化は一回しか行わない
  //
  //
  //キーパー用のaction
  keeper_ = std::make_shared<action::move>(world_, is_yellow_, keeper_id_);

  //壁用のaction
  for (auto it : wall_ids_) {
    wall_.emplace_back(std::make_shared<action::vec>(world_, is_yellow_, it));
  }
  target_x.resize(wall_.size());
  target_y.resize(wall_.size());
}

std::vector<std::shared_ptr<action::base>> defense::execute() {
  using boost::math::constants::pi;

  //ボールの座標
  const auto ball_x = world_.ball().x();
  const auto ball_y = world_.ball().y();

  //ゴールの座標
  const auto goal_x = world_.field().x_max();
  const auto goal_y = 0.0;

  //半径
  const auto radius = 1340.0;

  //基準座標を求めている
  //
  //いい感じのところ
  //
  //
  //比
  {
    const auto length = std::hypot(goal_x - ball_x, goal_y - ball_y); //ボール<->ゴール

    const auto ratio = radius / length; //全体に対しての基準座標の比

    x_ = (1 - ratio) * goal_x + ratio * ball_x;
    y_ = (1 - ratio) * goal_y + ratio * ball_y;
  }

  //ここから壁の処理
  //
  //
  //
  //
  //
  {
    //壁のイテレータ
    auto wall_it = wall_.begin();

    {
      auto target_x_it = target_x.begin();
      auto target_y_it = target_y.begin();
      //基準点からどれだけずらすか
      auto shift_ = 0.0;

      if (wall_.size() % 2) { //奇数
        (*target_x_it++) = x_;
        (*target_y_it++) = y_;

        shift_ = 180.0;
      } else { //偶数
        shift_ = 210.0;
      }

      //移動した量
      const auto move_x = ball_x;
      const auto move_y = ball_y;

      //計算の為に中心にずらした場合の座標
      const auto after_ball_x = ball_x - move_x;
      const auto after_ball_y = ball_y - move_y;

      auto after_base_x = x_ - move_x;
      auto after_base_y = y_ - move_y;

      // x軸から角度
      const auto alpha = util::wrap_to_2pi(
          std::atan2(after_base_y - after_ball_y, after_base_x - after_ball_x));

      after_base_x = std::hypot(after_base_x - after_ball_x, after_base_y - after_ball_y);
      after_base_y = 0.0;

      for (auto shift = shift_; wall_it != wall_.end(); shift += shift_, wall_it++) {
        //移動した先での仮の座標
        const auto tmp_x  = after_base_x;
        const auto tmp_y1 = std::sqrt(std::pow(shift, 2) + std::pow(after_base_x, 2) -
                                      std::pow(after_base_x, 2));
        const auto tmp_y2 = tmp_y1 * (-1);

        //回転した後の正しい座標
        const auto c_x1 = tmp_x * std::cos(alpha) - tmp_y1 * std::sin(alpha) + move_x;
        const auto c_y1 = tmp_x * std::sin(alpha) + tmp_y1 * std::cos(alpha) + move_y;
        const auto c_x2 = tmp_x * std::cos(alpha) - tmp_y2 * std::sin(alpha) + move_x;
        const auto c_y2 = tmp_x * std::sin(alpha) + tmp_y2 * std::cos(alpha) + move_y;

        (*target_x_it++) = c_x1;
        (*target_y_it++) = c_y1;
        (*target_x_it++) = c_x2;
        (*target_y_it++) = c_y2;
      }
    }
    {
      const auto wall_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
      auto wall_ids_it       = wall_ids_.begin();

      auto target_x_it = target_x.begin();
      auto target_y_it = target_y.begin();

      for (auto wall_it : wall_) {
        const auto wall_robot = wall_robots.at((*wall_ids_it++));
        const auto wall_x     = wall_robot.x();
        const auto wall_y     = wall_robot.y();
        const auto wall_theta = util::wrap_to_2pi(wall_robot.theta());

        //移動目標
        const auto sign_x = ((*target_x_it) - wall_x) * 6.0;
        const auto sign_y = ((*target_y_it) - wall_y) * 6.0;

        //ボールの向きを向くために,ゴール<->ボールの角度-自身の角度 をしてそれを角速度とする.
        const auto omega =
            util::wrap_to_2pi(std::atan2(ball_y - goal_y, ball_x - goal_x)) - wall_theta;
        wall_it->move_to(sign_x, sign_y, omega);

        target_y_it++;
        target_x_it++;
      }
    }
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
  {
    const auto demarcation = 2250.0; //縄張りの大きさ

    auto keeper_y     = 0.0;
    auto keeper_x     = 0.0;
    auto keeper_theta = 0.0;

    if (std::signbit(ball_x) == std::signbit(goal_x * (-1))) { // A
      //ゴール直前でボールに併せて横移動

      keeper_x = goal_x + (std::signbit(world_.field().x_max()) ? 100.0 : -100.0);
      keeper_y = ((ball_y >= -500 && ball_y <= 500) ? ball_y : 0);

    } else if (std::signbit(std::pow(ball_x - world_.field().x_max(), 2) + std::pow(ball_y, 2) -
                            std::pow(demarcation, 2))) { // C
      //ゴール前でディフェンスする

      {
        //ゴール前で張ってるキーパの位置
        const auto length = std::hypot(goal_x - ball_x, ball_y); //ゴール<->ボール
        const auto ratio  = (410) / length; //全体に対してのキーパー位置の比

        keeper_x = ((1 - ratio) * goal_x + ratio * ball_x);
        keeper_y = ratio * ball_y;
      }
      //もし支線の先にロボットがいたら
      //そのロボットの視線の先に移動
      //セット仕直し
    } else { // B
             //壁のすぐ後ろで待機
      auto shift = 0.0;
      if (y_ > 250) {
        shift = 250;
      } else if (y_ < -250) {
        shift = -250;
      }
      //基準点からちょっと下がったキーパの位置
      const auto length = std::hypot(goal_x - ball_x, shift - ball_y); //基準点<->ボール
      const auto ratio  = (910) / length; //全体に対してのキーパー位置の比

      keeper_x = (1 - ratio) * goal_x + ratio * ball_x;
      keeper_y = (1 - ratio) * shift + ratio * ball_y;

      if (keeper_y >= -250 &&
          keeper_y <= 250) { //もし基準座標が直線の範囲だったら直線に叩き込む
        keeper_x = (goal_x + (std::signbit(goal_x) ? 910 : -910));
      }
    }
    keeper_theta = util::wrap_to_2pi(std::atan2(ball_y - keeper_y, ball_x - keeper_x));

    keeper_->move_to(keeper_x, keeper_y, keeper_theta); //置く場所をセット
  }
  std::vector<std::shared_ptr<action::base>> re_wall{
      wall_.begin(), wall_.end()}; //型を合わせるために無理矢理作り直す
  re_wall.push_back(keeper_);      //配列を返すためにキーパーを統合する
  return re_wall;                  //返す
}
}
}
}
