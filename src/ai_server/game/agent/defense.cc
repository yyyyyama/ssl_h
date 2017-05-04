#include <cmath>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

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
      orientation_(Eigen::Vector2d::Zero()),
      mode_(defense_mode::normal_mode) {
  // actionの生成部分
  //
  // actionの初期化は一回しか行わない
  //
  //
  //キーパー用のaction
  //移動用
  keeper_v_ = std::make_shared<action::vec>(world_, is_yellow_, keeper_id_);
  keeper_k_ = std::make_shared<action::kick_action>(world_, is_yellow_, keeper_id_);

  //壁用のaction
  for (auto it : wall_ids_) {
    wall_.emplace_back(std::make_shared<action::vec>(world_, is_yellow_, it));
  }
  target_.resize(wall_.size());
}

void defense::set_mode(defense_mode mode) {
  mode_ = mode;
}

std::vector<std::shared_ptr<action::base>> defense::execute() {
  using boost::math::constants::pi;

  //ボールの座標
  const Eigen::Vector2d ball(world_.ball().x(), world_.ball().y());

  //ボールがゴールより後ろに来たら現状維持
  if (ball.x() < -4500.0 || ball.x() > 4500.0) {
    for (auto wall_it : wall_) {
      wall_it->move_to(0.0, 0.0, 0.0);
    }
    keeper_v_->move_to(0.0, 0.0, 0.0);

    std::vector<std::shared_ptr<action::base>> re_wall{
        wall_.begin(), wall_.end()}; //型を合わせるために無理矢理作り直す
    re_wall.push_back(keeper_v_);    //配列を返すためにキーパーを統合する

    return re_wall;
  }
  //ゴールの座標
   const Eigen::Vector2d goal(world_.field().x_min(), 0.0);

  const auto ball_theta =
      std::signbit(goal.x())
          ? std::atan2(ball.y() - goal.y(), ball.x() - goal.x())
          : util::wrap_to_2pi(std::atan2(ball.y() - goal.y(), ball.x() - goal.x()));
  //半径
  const auto radius = 1400.0;

  //基準座標を求めている
  //
  //いい感じのところ
  //
  //
  //比
  {
    const auto length = (goal - ball).norm(); //ボール<->ゴール

    const auto ratio = radius / length; //全体に対しての基準座標の比

    orientation_ = (1 - ratio) * goal + ratio * ball;
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

      const auto demarcation = 2500.0; //縄張りの大きさ

      if (wall_.size() % 2) { //奇数
        (*target_it++) = orientation_;
        wall_it++;
        shift_ = 400.0;
        if (std::signbit(std::pow(ball.x() - goal.x(), 2) + std::pow(ball.y(), 2) -
                         std::pow(demarcation, 2))) {
          shift_ = 200;
        }
      } else { //偶数
        shift_ = 210.0;
        if (std::signbit(std::pow(ball.x() - goal.x(), 2) + std::pow(ball.y(), 2) -
                         std::pow(demarcation, 2))) {
          shift_ = 110;
        }
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

      for (auto shift = shift_; wall_it != wall_.end(); shift += (shift_ * 2), wall_it += 2) {
        //移動した先での仮の座標
        const Eigen::Vector2d tmp1(after_base.x(), shift);
        const Eigen::Vector2d tmp2(tmp1.x(), tmp1.y() * (-1));

        //回転した後の正しい座標
        (*target_it++) = (rotate * tmp1) + move;
        (*target_it++) = (rotate * tmp2) + move;

        //もしディフェンスエリアないに入ってしまっても順番が入れ替わらないようにする
        if ((std::pow(ball.x() - goal.x(), 2) + std::pow(ball.y() - goal.y(), 2) -
             std::pow(1400.0, 2)) < 0) {
          const auto tmp   = *(target_it - 2);
          *(target_it - 2) = *(target_it - 1);
          *(target_it - 1) = tmp;
        }
      }
    }
    {
      switch (mode_) {
        case defense_mode::normal_mode: {
          const auto wall_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
          auto wall_ids_it       = wall_ids_.begin();

          auto target_it = target_.begin();

          for (auto wall_it : wall_) {
            const auto wall_robot = wall_robots.at((*wall_ids_it++));
            const Eigen::Vector2d wall(wall_robot.x(), wall_robot.y());
            const auto wall_theta = std::signbit(goal.x())
                                        ? wall_robot.theta()
                                        : util::wrap_to_2pi(wall_robot.theta());

            //移動目標
            const Eigen::Vector2d sign(((*target_it) - wall) * 6.0);

            //ボールの向きを向くために,ゴール<->ボールの角度-自身の角度
            //をしてそれを角速度とする.
            // const auto omega =
            //    util::wrap_to_2pi(std::atan2(ball.y() - goal.y(), ball.x() - goal.x())) -
            //    wall_theta;
            const auto omega = ball_theta - wall_theta;
            std::cout << "ball_theta : " << wall_theta << " wall_theta : " << wall_theta
                      << std::endl;
            wall_it->move_to(sign.x(), sign.y(), omega);
            target_it++;
          }
          break;
        }
        case defense_mode::pk_mode: {
          for (auto wall_it : wall_) {
            wall_it->move_to(0.0, 0.0, 0.0);
          }
          break;
        }
      }
    }
    std::vector<std::shared_ptr<action::base>> re_wall{
        wall_.begin(), wall_.end()}; //型を合わせるために無理矢理作り直す
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
      Eigen::Vector2d keeper(Eigen::Vector2d::Zero());
      const auto my_robots    = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
      const auto keeper_robot = my_robots.at(keeper_id_);
      //キーパーの角度
      const auto keeper_theta = std::signbit(goal.x())
                                    ? keeper_robot.theta()
                                    : util::wrap_to_2pi(keeper_robot.theta());
      const Eigen::Vector2d keeper_c(keeper_robot.x(), keeper_robot.y());
      //速さに掛ける係数
      Eigen::Vector2d coefficient(Eigen::Vector2d::Zero());
      switch (mode_) {
        case defense_mode::normal_mode: {
          const auto demarcation = 2500.0; //縄張りの大きさ
          //めっちゃ近くにボールがあったら蹴ってみたいね
          if (std::signbit(std::pow(ball.x() - goal.x(), 2) + std::pow(ball.y() - goal.y(), 2) -
                           std::pow(600, 2))) {
            if (std::signbit(std::pow(ball.x() - keeper_robot.x(), 2) +
                             std::pow(ball.y() - keeper_robot.y(), 2) - std::pow(120, 2)) &&
                (util::wrap_to_2pi(std::atan2(ball.y() - goal.y(), ball.x() - goal.x())) -
                 keeper_theta) < 0.5) {
              if (!keeper_k_->finished()) {
                keeper_k_->kick_to(goal.x() * (-1), goal.y());
                keeper_k_->set_kick_type({model::command::kick_type_t::chip, 10.0});
              }
              re_wall.push_back(keeper_k_); //配列を返すためにキーパーを統合する
            }
          } else {
            if (std::signbit(goal.x() * (-1)) ==
                std::signbit((ball.x() + (std::signbit(goal.x() * (-1)) ? 500 : -500)))) { // A
              //ゴール直前でボールに併せて横移動

              keeper.x()  = goal.x() + (std::signbit(goal.x()) ? 110.0 : -110.0);
              keeper.y()  = ((ball.y() >= -500 && ball.y() <= 500) ? ball.y() : 0);
              coefficient = {0.7, 7.0};

            } else if (std::signbit(std::pow(ball.x() - goal.x(), 2) +
                                    std::pow(ball.y() - goal.y(), 2) -
                                    std::pow(demarcation, 2))) { // C
              //ゴール前でディフェンスする

              {
                //ゴール前で張ってるキーパの位置
                const auto length = std::hypot(goal.x() - ball.x(), ball.y()); //ゴール<->ボール
                const auto ratio = (410) / length; //全体に対してのキーパー位置の比

                keeper = (1 - ratio) * goal + ratio * ball;
              }
              coefficient = {6.0, 7.0};
            } else { // B
                     //壁のすぐ後ろで待機
              auto shift = 0.0;
              if (orientation_.y() > 250) {
                shift = 250;
              } else if (orientation_.y() < -250) {
                shift = -250;
              }
              //基準点からちょっと下がったキーパの位置
              const auto length =
                  std::hypot(goal.x() - ball.x(), shift - ball.y()); //基準点<->ボール
              const auto ratio = (910) / length; //全体に対してのキーパー位置の比

              keeper.x() = (1 - ratio) * goal.x() + ratio * ball.x();
              keeper.y() = (1 - ratio) * shift + ratio * ball.y();

              if (keeper.y() >= -250 &&
                  keeper.y() <= 250) { //もし基準座標が直線の範囲だったら直線に叩き込む
                keeper.x() = (goal.x() + (std::signbit(goal.x()) ? 910 : -910));
              }
              coefficient = {6.5, 7.0};
            }
          }
          break;
        }
        case defense_mode::pk_mode: {
          const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();

          coefficient = {7.0, 7.0};
          //敵のシューターを線形探索する
          //
          //
          //ボールにもっとも近いやつがシューターだと仮定
          //

          //暫定的なボールに最も近い敵ロボット
          unsigned int enemy_robot_id = 0;

          //最小値を比べるための変数
          auto min_val = 0xffffffff;

          for (auto enemy_it : enemy_robots) {
            auto len =
                std::hypot((enemy_it.second).x() - ball.x(), (enemy_it.second).y() - ball.y());
            if (len < min_val) {
              min_val        = len;
              enemy_robot_id = (enemy_it.first);
            }
          }

          //キーパーのy座標は敵シューターの視線の先
          keeper.y() =
              1000.0 * std::tan(util::wrap_to_2pi(enemy_robots.at(enemy_robot_id).theta()));
          if (keeper.y() > 410) {
            keeper.y() = 410;
          } else if (keeper.y() < -410) {
            keeper.y() = -410;
          }
          //自チームゴールが正か負かで向きが反転するので直す
          keeper.y() *= (std::signbit(goal.x()) ? -1 : 1);

          keeper.x() = goal.x() + (std::signbit(goal.x()) ? 110.0 : -110.0);
          break;
        }
      }
      //移動目標
      const Eigen::Vector2d sign((keeper.x() - keeper_c.x()) * coefficient.x(),
                                 (keeper.y() - keeper_c.y()) * coefficient.y());

      //ボールの向きを向くために,ゴール<->ボールの角度-自身の角度 をしてそれを角速度とする.
      const auto omega = ball_theta - keeper_theta;
      keeper_v_->move_to(sign.x(), sign.y(), omega);

      re_wall.push_back(keeper_v_); //配列を返すためにキーパーを統合する
    }

    return re_wall; //返す
  }
}
}
}
}
