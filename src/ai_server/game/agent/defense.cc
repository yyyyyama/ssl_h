#include <iostream>
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
      orientation_(Eigen::Vector2d::Zero()),
      mode_(defense_mode::normal_mode) {
  // actionの生成部分
  //
  // actionの初期化は一回しか行わない
  //
  //
  //キーパー用のaction
  //移動用
  keeper_ = std::make_shared<action::vec>(world_, is_yellow_, keeper_id_);

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
    keeper_->move_to(0.0, 0.0, 0.0);

    std::vector<std::shared_ptr<action::base>> re_wall{
        wall_.begin(), wall_.end()}; //型を合わせるために無理矢理作り直す
    re_wall.push_back(keeper_);      //配列を返すためにキーパーを統合する

    return re_wall;
  }
  //ゴールの座標
  const Eigen::Vector2d goal(world_.field().x_min(), 0.0);
  //  const Eigen::Vector2d goal(4500.0, 0.0);

  const auto ball_theta = std::atan2(ball.y() - goal.y(), ball.x() - goal.x());
  //      std::signbit(goal.x())
  //          ? std::atan2(ball.y() - goal.y(), ball.x() - goal.x())
  //          : util::wrap_to_2pi(std::atan2(ball.y() - goal.y(), ball.x() - goal.x()));

  //基準座標を求めている
  //
  //いい感じのところ
  //
  //
  //比
  {
    //半径
    const auto radius = 1400.0;

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

      const auto demarcation = 2000.0; //縄張りの大きさ
      //壁の数によってずらしていく倍率が変わるのでその倍率
      auto magnification = 0.0;
      Eigen::Vector2d odd{0.0, 0.0};

      if (wall_.size() % 2) { //奇数
        {
          const auto length = (orientation_ - ball).norm(); //ボール<->ゴール

          const auto ratio = 400 / length; //全体に対しての基準座標の比

          odd = (1 - ratio) * orientation_ + ratio * ball;
        }
        if (odd.x() < orientation_.x()) {
          {
            const auto length = (orientation_ - ball).norm(); //ボール<->ゴール

            const auto ratio = -400 / length; //全体に対しての基準座標の比

            odd = (1 - ratio) * orientation_ + ratio * ball;
          }
        }
        (*target_it++) = odd;
        wall_it++;
        magnification = 1.0;
        shift_        = 200.0;
        if (std::signbit(std::pow(ball.x() - goal.x(), 2) + std::pow(ball.y(), 2) -
                         std::pow(demarcation, 2))) {
          *(target_it - 1) = orientation_;
          shift_           = 200;
        } else {
        }
      } else { //偶数
        magnification = 2.0;
        shift_        = 190.0;
        if (std::signbit(std::pow(ball.x() - goal.x(), 2) + std::pow(ball.y(), 2) -
                         std::pow(demarcation, 2))) {
          shift_ = 90;
        } else {
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

      after_base.x() = (after_base - after_ball).norm();
      after_base.y() = 0.0;

      for (auto shift = shift_; wall_it != wall_.end();
           shift += (shift_ * magnification), wall_it += 2) {
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
    //ワンツー対策
    {
      if (std::signbit(std::pow(ball.x() - goal.x(), 2) + std::pow(ball.y(), 2) -
                       std::pow(3000.0, 2))) {
        const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();

        //暫定的なボールに最も近い敵ロボット
        unsigned int enemy_robot_id = 0;
        {
          for (auto enemy_it : enemy_robots) {
            std::hypot((enemy_it.second).x() - ball.x(), (enemy_it.second).y() - ball.y());
            if (std::pow(ball.x() - (enemy_it.second).x(), 2) +
                    std::pow(ball.y() - (enemy_it.second).y(), 2) - std::pow(100.0, 2) <
                0) {
              enemy_robot_id = (enemy_it.first);
            }
          }
        }

        if (enemy_robots.count(enemy_robot_id)) {
          const auto& enemy_robot = enemy_robots.at(enemy_robot_id);

          const auto enemy_theta = util::wrap_to_pi(enemy_robot.theta());
          const Eigen::Vector2d enemy_pos(enemy_robot.x(), enemy_robot.y());

          //暫定的な受け取り敵ロボット
          unsigned int recerve_id = 0xff;

          for (auto enemy_it : enemy_robots) {
            if ((enemy_it.first) == enemy_robot_id) {
              continue;
            }
            const Eigen::Vector2d comp_pos((enemy_it.second).x(), (enemy_it.second).y());
            const auto comp_theta =
                std::atan2(comp_pos.x() - enemy_pos.x(), comp_pos.y() - enemy_pos.y()) -
                (std::signbit(enemy_pos.y()) ? 0 : pi<double>());

            if ((comp_theta - (pi<double>() / 18.0)) < enemy_theta &&
                (comp_theta + (pi<double>() / 18.0)) > enemy_theta) {
              recerve_id = (enemy_it.first);
            }
          }
          {
            if (recerve_id != 0xff) {
              if (enemy_robots.count(recerve_id)) {
                const auto& recerve_robot = enemy_robots.at(recerve_id);
                const Eigen::Vector2d recerve_pos(recerve_robot.x(), recerve_robot.y());
                Eigen::Vector2d new_pos;
                {
                  //半径
                  const auto radius = 1400.0;

                  const auto length = (goal - recerve_pos).norm(); //レシーバー<->ゴール

                  const auto ratio = radius / length; //全体に対しての基準座標の比

                  new_pos = (1 - ratio) * goal + ratio * recerve_pos;
                }
                //近い奴のいてレーたを格納する
                auto it = target_.begin();
                {
                  //最小値を比べるための変数
                  auto min_val   = 0xffffffff;
                  auto target_it = target_.begin();

                  for (auto wall_ids_it : wall_ids_) {
                    const auto wall_robots =
                        is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
                    if (wall_robots.count(wall_ids_it)) {
                      const auto& wall_robot = wall_robots.at(wall_ids_it);
                      const Eigen::Vector2d wall(wall_robot.x(), wall_robot.y());
                      if ((new_pos - wall).norm() < min_val) {
                        it      = target_it;
                        min_val = (new_pos - wall).norm();
                      }
                      target_it++;
                    }
                  }
                }
                (*it) = new_pos;
              }
            }
          }
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
            if (wall_robots.count((*wall_ids_it))) {
              const auto& wall_robot = wall_robots.at((*wall_ids_it++));
              const Eigen::Vector2d wall(wall_robot.x(), wall_robot.y());
              const auto wall_theta = util::wrap_to_pi(wall_robot.theta());

              //移動目標
              auto coefficient = 20.0;
              if ((((*target_it) - wall) * coefficient).norm() > 1400.0) {
                coefficient = 3.0;
              }
              const Eigen::Vector2d sign(((*target_it) - wall) * coefficient);

              //ボールの向きを向くために,ゴール<->ボールの角度-自身の角度をしてそれを角速度とする.
              const auto omega = ball_theta - wall_theta;
              wall_it->move_to(sign.x(), sign.y(), omega);
              target_it++;

            } else {
              wall_it->move_to(0.0, 0.0, 0.0);
            }
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
    const auto my_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
    if (my_robots.count(keeper_id_)) {
      const auto& keeper_robot = my_robots.at(keeper_id_);
      //キーパーの角度
      const auto keeper_theta = util::wrap_to_pi(keeper_robot.theta());
      const Eigen::Vector2d keeper_c(keeper_robot.x(), keeper_robot.y());
      //速さに掛ける係数
      //      Eigen::Vector2d coefficient(Eigen::Vector2d::Zero());
      switch (mode_) {
        case defense_mode::normal_mode: {
          /*const auto demarcation = 2500.0; //縄張りの大きさ
    if (std::signbit(goal.x() * (-1)) ==
    std::signbit((ball.x() + (std::signbit(goal.x() * (-1)) ? 500 : -500)))) { // A
    //ゴール直前でボールに併せて横移動

    keeper.x()  = goal.x() + (std::signbit(goal.x()) ? 110.0 : -110.0);
    keeper.y()  = ((ball.y() >= -500 && ball.y() <= 500) ? ball.y() : 0);
    coefficient = {0.7, 7.0};

    }else if (std::signbit(std::pow(ball.x() - goal.x(), 2) +
    std::pow(ball.y() - goal.y(), 2) -
    std::pow(demarcation, 2))) { // C
    //ゴール前でディフェンスする

    {
    //ゴール前で張ってるキーパの位置
    const auto length = std::hypot(goal.x() - ball.x(), ball.y()); //ゴール<->ボール
    const auto ratio = (410) / length; //全体に対してのキーパー位置の比

    keeper = (1 - ratio) * goal + ratio * ball;
    }
    coefficient = {5.5, 7.0};
    } else { // B*/
          //壁のすぐ後ろで待機
          //基準点からちょっと下がったキーパの位置
          const auto length =
              std::hypot(goal.x() - ball.x(), goal.y() - ball.y()); //基準点<->ボール
          const auto ratio = (1250) / length; //全体に対してのキーパー位置の比

          keeper = (1 - ratio) * goal + ratio * ball;

          //  }
          break;
        }
        case defense_mode::pk_mode: {
          const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();

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
          if (enemy_robots.count(enemy_robot_id)) {
            keeper.y() = (1000.0 * std::tan(enemy_robots.at(enemy_robot_id).theta())) * (-1);
          } else {
            keeper.y() = keeper_c.y();
          }

          if (keeper.y() > 410) {
            keeper.y() = 410;
          } else if (keeper.y() < -410) {
            keeper.y() = -410;
          }

          keeper.x() = goal.x() + 110.0;
          break;
        }
      }
      //移動目標
      auto coefficient = 20.0;
      if (((keeper - keeper_c) * coefficient).norm() > 1400.0) {
        coefficient = 3.0;
      }

      const Eigen::Vector2d sign((keeper.x() - keeper_c.x()) * coefficient,
                                 (keeper.y() - keeper_c.y()) * coefficient);

      //ボールの向きを向くために,ゴール<->ボールの角度-自身の角度
      //をしてそれを角速度とする.
      const auto omega = ball_theta - keeper_theta;
      keeper_->move_to(sign.x(), sign.y(), omega);

      re_wall.push_back(keeper_); //配列を返すためにキーパーを統合する
    } else {
      keeper_->move_to(0.0, 0.0, 0.0);
    }
  }

  return re_wall; //返す
}
}
}
}
