#include <cmath>

#include "ai_server/game/agent/defense.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace agent {

defense::defense(const model::world& world, bool is_yellow, unsigned int keeper_id,
                 const std::vector<unsigned int>& wall_ids)
    : base(world, is_yellow),
      keeper_id_(keeper_id),
      wall_ids_(wall_ids),
      orientation_(Eigen::Vector2d::Zero()) {
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
  target_.resize(wall_.size());
}

std::vector<std::shared_ptr<action::base>> defense::execute() {
  using boost::math::constants::pi;

  //ボールの座標
  const Eigen::Vector2d ball(world_.ball().x(), world_.ball().y());

  //ゴールの座標
  const Eigen::Vector2d goal(world_.field().x_max(), 0.0);

  //半径
  const auto radius = 1340.0;

  //基準座標を求めている
  //
  //いい感じのところ
  //
  //
  //比
  {
    const auto length = std::hypot(goal.x() - ball.x(), goal.y() - ball.y()); //ボール<->ゴール

    const auto ratio = radius / length; //全体に対しての基準座標の比

    orientation_.x() = (1 - ratio) * goal.x() + ratio * ball.x();
    orientation_.y() = (1 - ratio) * goal.y() + ratio * ball.y();
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
      auto target_it = target_.begin();
      //基準点からどれだけずらすか
      auto shift_ = 0.0;

      if (wall_.size() % 2) { //奇数
        (*target_it++) = {orientation_.x(), orientation_.y()};
        wall_it++;
        shift_ = 180.0;
      } else { //偶数
        shift_ = 210.0;
      }

      //移動した量
      const Eigen::Vector2d move(ball.x(), ball.y());

      //計算の為に中心にずらした場合の座標
      Eigen::Vector2d after_ball(ball.x() - move.x(), ball.y() - move.y());

      Eigen::Vector2d after_base(orientation_.x() - move.x(), orientation_.y() - move.y());

      // x軸から角度
      const auto alpha = util::wrap_to_2pi(
          std::atan2(after_base.y() - after_ball.y(), after_base.x() - after_ball.x()));
      //回転行列
      // const Eigen::Matrix2d rotate = Eigen::Rotation2Dd(alpha);
      const Eigen::Rotation2Dd rotate(alpha);

      after_base.x() =
          std::hypot(after_base.x() - after_ball.x(), after_base.y() - after_ball.y());
      after_base.y() = 0.0;

      for (auto shift = shift_; wall_it != wall_.end(); shift += shift_, wall_it += 2) {
        //移動した先での仮の座標
        const Eigen::Vector2d tmp1(after_base.x(), shift);
        const Eigen::Vector2d tmp2(tmp1.x(), tmp1.y() * (-1));

        //回転した後の正しい座標
        Eigen::Vector2d c1 = rotate * tmp1;
        c1                 = {c1.x() + move.x(), c1.y() + move.y()};
        Eigen::Vector2d c2 = rotate * tmp2;
        c2                 = {c2.x() + move.x(), c2.y() + move.y()};
        (*target_it++)     = {c1.x(), c1.y()};
        (*target_it++)     = {c2.x(), c2.y()};
      }
    }
    {
      const auto wall_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
      auto wall_ids_it       = wall_ids_.begin();

      auto target_it = target_.begin();

      for (auto wall_it : wall_) {
        const auto wall_robot = wall_robots.at((*wall_ids_it++));
        const Eigen::Vector2d wall(wall_robot.x(), wall_robot.y());
        const auto wall_theta = util::wrap_to_2pi(wall_robot.theta());

        //移動目標
        const Eigen::Vector2d sign(((*target_it).x() - wall.x()) * 6.0,
                                   ((*target_it).y() - wall.y()) * 6.0);

        //ボールの向きを向くために,ゴール<->ボールの角度-自身の角度 をしてそれを角速度とする.
        const auto omega =
            util::wrap_to_2pi(std::atan2(ball.y() - goal.y(), ball.x() - goal.x())) -
            wall_theta;
        wall_it->move_to(sign.x(), sign.y(), omega);
        target_it++;
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

    Eigen::Vector2d keeper;
    keeper            = Eigen::Vector2d::Zero();
    auto keeper_theta = 0.0;

    if (std::signbit(ball.x()) == std::signbit(goal.x() * (-1))) { // A
      //ゴール直前でボールに併せて横移動

      keeper.x() = goal.x() + (std::signbit(goal.x() ? 100.0 : -100.0));
      keeper.y() = ((ball.y() >= -500 && ball.y() <= 500) ? ball.y() : 0);

    } else if (std::signbit(std::pow(ball.x() - goal.x(), 2) + std::pow(ball.y(), 2) -
                            std::pow(demarcation, 2))) { // C
      //ゴール前でディフェンスする

      {
        //ゴール前で張ってるキーパの位置
        const auto length = std::hypot(goal.x() - ball.x(), ball.y()); //ゴール<->ボール
        const auto ratio  = (410) / length; //全体に対してのキーパー位置の比

        keeper.x() = ((1 - ratio) * goal.x() + ratio * ball.x());
        keeper.y() = ratio * ball.y();
      }
    } else { // B
             //壁のすぐ後ろで待機
      auto shift = 0.0;
      if (orientation_.y() > 250) {
        shift = 250;
      } else if (orientation_.y() < -250) {
        shift = -250;
      }
      //基準点からちょっと下がったキーパの位置
      const auto length = std::hypot(goal.x() - ball.x(), shift - ball.y()); //基準点<->ボール
      const auto ratio = (910) / length; //全体に対してのキーパー位置の比

      keeper.x() = (1 - ratio) * goal.x() + ratio * ball.x();
      keeper.y() = (1 - ratio) * shift + ratio * ball.y();

      if (keeper.y() >= -250 &&
          keeper.y() <= 250) { //もし基準座標が直線の範囲だったら直線に叩き込む
        keeper.x() = (goal.x() + (std::signbit(goal.x()) ? 910 : -910));
      }
    }
    keeper_theta = util::wrap_to_2pi(std::atan2(ball.y() - keeper.y(), ball.x() - keeper.x()));

    keeper_->move_to(keeper.x(), keeper.y(), keeper_theta); //置く場所をセット
  }
  std::vector<std::shared_ptr<action::base>> re_wall{
      wall_.begin(), wall_.end()}; //型を合わせるために無理矢理作り直す
  re_wall.push_back(keeper_);      //配列を返すためにキーパーを統合する
  return re_wall;                  //返す
}
}
}
}
