#include <cmath>
#include <iostream>

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
    wall_.emplace_back(std::make_shared<action::move>(world_, is_yellow_, it));
  }
  target_x.resize(wall_.size());
  target_y.resize(wall_.size());
}

std::vector<std::shared_ptr<action::base>> defense::execute() {
  using boost::math::constants::pi;

  //ボールの座標
  const auto ball_x = world_.ball().x();
  const auto ball_y = world_.ball().y();

  std::cout << "ball_x : " << ball_x << " ball_y : " << ball_y << std::endl;
  //ゴールの座標
  auto goal_x       = world_.field().x_max();
  const auto goal_y = 0.0;

  goal_x = 4500;
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

  if (y_ >= -250 && y_ <= 250) { //もし基準座標が直線の範囲だったら直線に叩き込む
    x_ = (goal_x + (std::signbit(goal_x) ? 1100 : -1100));
  }

  //ここから壁の処理
  //
  //
  //
  //
  //
  {
    //基準点からボールへの向き
    const auto theta = util::wrap_to_2pi(std::atan2(ball_y - y_, ball_x - x_));

    //壁のイテレータ
    auto wall_it = wall_.begin();

    {
      auto target_x_it = target_x.begin();
      auto target_y_it = target_y.begin();
      //基準点からどれだけずらすか
      auto shift_ = 0.0;

      if (wall_.size() % 2) { //奇数
                              //    (*wall_it++)->move_to(x_, y_, theta);
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

        //    (*wall_it)->move_to(c_x1, c_y1, theta);
        //     wall_it++;
        ///    (*wall_it)->move_to(c_x2, c_y2, theta);
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

        std::cout << "wall_x : " << wall_x << " wall_y : " << wall_y << std::endl;

        std::cout << "target_x : " << *target_x_it << " target_y : " << *target_y_it
                  << std::endl;

        //目的地との直線上に交点がなかったらそのまま移動
        const auto slope   = ((*target_y_it) - wall_y) / ((*target_x_it) - wall_x);
        const auto segment = wall_y - slope * wall_x;

        //交点の数を判別式で出している.
        //
        //決して適当にa,b,c,Dって使ってるわけじゃないよ？意味あるからね
        //
        const auto a = (1 + std::pow(slope, 2));
        const auto b = (2 * slope * segment - 2 * goal_x - 2 * goal_y * slope);
        const auto c =
            (std::pow(goal_x, 2) + std::pow(segment, 2) - 2 * segment * std::pow(goal_y, 2) +
             std::pow(goal_y, 2) - std::pow(1250, 2));
        const auto D = std::pow(b, 2) - 4 * a * c;

        // wall_it->move_to((*target_x_it++),(*target_y_it++),theta);

        if (D > 0) {
          //現在地と目的地の距離に応じて分け方を変える
          const auto L = std::hypot((*target_y_it) - wall_y, (*target_x_it) - wall_x);
          //幸福ディバイド
          //実際に試して問題なかった値を採用
          auto divided = 0.0;
          if (L > 1700) {
            divided = 1.0 / 3.0;
          } else if (L > 900) {
            divided = 1.0 / 2.0;
          } else {
            divided = 1.0 / 1.0;
          }
          //ディフェンスラインからどれだけずらすか
          const auto shift = 1200;

          const auto index_x = (1.0 - divided) * wall_x + divided * (*target_x_it);
          const auto index_y = (1.0 - divided) * wall_y + divided * (*target_y_it);

          std::cout << "index_x : " << index_x << " index_y : " << index_y << std::endl;

          if (index_y > 250) {
            const auto length = std::hypot(index_x - goal_x, index_y - 250); //中心<->index

            const auto ratio  = 1 - (length / shift); //目的<->indexの比
            const auto sign_x = (-ratio * goal_x + 1 * index_x) / (length / shift);
            const auto sign_y = (-ratio * 250 + 1 * index_y) / (length / shift);
            std::cout << "sign_x : " << sign_x << " sign_y : " << sign_y << std::endl;

            wall_it->move_to(sign_x, sign_y, theta);

          } else if (index_y < -250) {
            const auto length = std::hypot(index_x - goal_x, index_y - (-250)); //中心<->index

            const auto ratio  = 1 - (length / shift); //目的<->indexの比
            const auto sign_x = (-ratio * goal_x + 1 * index_x) / (length / shift);
            const auto sign_y = (-ratio * (-250) + 1 * index_y) / (length / shift);
            std::cout << "sign_x : " << sign_x << " sign_y : " << sign_y << std::endl;

            wall_it->move_to(sign_x, sign_y, theta);

          } else {
            const auto sign_x = std::signbit(goal_x) ? -3400 : 3400;
            const auto sign_y = index_y;
            std::cout << "sign_x : " << sign_x << " sign_y : " << sign_y << std::endl;

            wall_it->move_to(sign_x, sign_y, theta);
          }
        } else {
          wall_it->move_to((*target_x_it), (*target_y_it), theta);
        }
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
        const auto ratio  = (400) / length; //全体に対してのキーパー位置の比

        keeper_x = ((1 - ratio) * goal_x + ratio * ball_x);
        keeper_y = ratio * ball_y;
      }
      //もし支線の先にロボットがいたら
      //そのロボットの視線の先に移動
      //セット仕直し
    } else { // B
      //壁のすぐ後ろで待機

      {
        //基準点からちょっと下がったキーパの位置
        const auto length = std::hypot(goal_x - ball_x, ball_y); //基準点<->ボール
        const auto ratio  = (800) / length; //全体に対してのキーパー位置の比

        keeper_x = (1 - ratio) * goal_x + ratio * ball_x;
        keeper_y = ratio * ball_y;
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
