#include <cmath>

#include "ai_server/game/agent/defense.h"
#include "ai_server/model/command.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace agent {

defense::defense(const model::world& world, bool is_yellow, unsigned int keeper_id,
                 const std::vector<unsigned int>& wall_ids,
                 const std::vector<unsigned int>& marking_ids)
    : base(world, is_yellow),
      keeper_id_(keeper_id),
      wall_ids_(wall_ids),
      marking_ids_(marking_ids),
      orientation_(Eigen::Vector2d::Zero()),
      mode_(defense_mode::normal_mode) {
  // actionの生成部分
  //
  // actionの初期化は一回しか行わない
  //
  //

  //キーパー用のaction
  keeper_ = std::make_shared<action::guard>(world_, is_yellow_, keeper_id_);

  //壁用のaction
  for (auto it : wall_ids_) {
    wall_.emplace_back(std::make_shared<action::guard>(world_, is_yellow_, it));
  }
  wall_target_.resize(wall_.size());

  //壁用のaction
  for (auto it : marking_ids_) {
    marking_.emplace_back(std::make_shared<action::marking>(world_, is_yellow_, it));
  }
}

void defense::set_mode(defense_mode mode) {
  mode_ = mode;
}

std::vector<std::shared_ptr<action::base>> defense::execute() {
  using boost::math::constants::pi;

  //ボールの座標
  const Eigen::Vector2d ball_v(world_.ball().vx(), world_.ball().vy());
  const Eigen::Vector2d ball_p(world_.ball().x(), world_.ball().y());
  const Eigen::Vector2d ball_k(ball_v * 0.5);
  const Eigen::Vector2d ball(ball_p + ball_k);

  //ボールがゴールより後ろに来たら現状維持
  if (ball.x() < world_.field().x_min() || ball.x() > world_.field().x_max()) {
    auto target_it = wall_target_.begin();
    for (auto wall_it : wall_) {
      wall_it->move_to((*target_it++).x(), (*target_it++).y(), 0.0);
    }
    keeper_->move_to(keeper_target_.x(), keeper_target_.y(), 0.0);

    std::vector<std::shared_ptr<action::base>> re_wall{
        wall_.begin(), wall_.end()}; //型を合わせるために無理矢理作り直す
    re_wall.push_back(keeper_);      //配列を返すためにキーパーを統合する

    return re_wall;
  }

  //ゴールの座標
  const Eigen::Vector2d goal(world_.field().x_min(), 0.0);

  const auto ball_theta = std::atan2(ball.y() - goal.y(), ball.x() - goal.x());

  //基準座標を求めている
  //
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

    std::vector<Eigen::Vector2d> target;
    //移動目標を出す
    {
      //基準点からどれだけずらすか
      auto shift_ = 0.0;
      //縄張りの大きさ
      const auto demarcation = 3000.0;
      //壁の数によってずらしていく倍率が変わるのでその倍率
      auto magnification = 0.0;

      if (wall_.size() % 2) { //奇数
        //奇数なら五稜郭っぽいことしたいね<-塹壕戦の基本！
        Eigen::Vector2d odd{Eigen::Vector2d::Zero()};
        //中央の壁は前に出ろ
        {
          const auto length = (orientation_ - ball).norm(); //ボール<->ゴール

          const auto ratio = 400 / length; //全体に対しての基準座標の比

          odd = ((1 - ratio) * orientation_ + ratio * ball);
        }
        //時々位置が反転するのでその処理
        {
          if (odd.x() < orientation_.x()) {
            const auto length = (orientation_ - ball).norm(); //ボール<->ゴール

            const auto ratio = -400 / length; //全体に対しての基準座標の比

            odd = (1 - ratio) * orientation_ + ratio * ball;
          }
        }

        //敵がめっちゃ近づいたら閉める
        if ((ball - goal).norm() < demarcation) {
          //前に出てる場合じゃねぇ
          odd = orientation_;
        }
        target.push_back(odd);
        wall_it++;
        magnification = 1.0;
        shift_        = 200;
      } else { //偶数
        magnification = 2.0;
        shift_        = 190.0;
        //敵がめっちゃ近づいたら閉める
        if ((ball - goal).norm() < demarcation) {
          shift_ = 90;
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
      const Eigen::Rotation2Dd rotate(alpha);

      after_base.x() = (after_base - after_ball).norm();
      after_base.y() = 0.0;

      for (auto shift = shift_; wall_it != wall_.end();
           shift += (shift_ * magnification), wall_it += 2) {
        //移動した先での仮の座標
        const Eigen::Vector2d tmp1(after_base.x(), shift);
        const Eigen::Vector2d tmp2(tmp1.x(), tmp1.y() * (-1));

        //回転した後の正しい座標
        target.push_back((rotate * tmp1) + move);
        target.push_back((rotate * tmp2) + move);

        //もしディフェンスエリアないに入ってしまっても順番が入れ替わらないようにする
        if ((ball - goal).norm() < 1400) {
          const auto tmp = target.back();
          target.pop_back();
          auto target_it = target.end();
          target.insert(--target_it, tmp);
        }
      }
    }

    //実際にアクションを詰めて返す
    {
      auto target_it     = target.begin();
      auto old_target_it = wall_target_.begin();
      for (auto wall_it : wall_) {
        wall_it->move_to((*target_it).x(), (*target_it).y(), ball_theta);
        wall_it->set_kick_type({model::command::kick_type_t::chip, 255});
        wall_it->set_dribble(3);
        (*old_target_it++) = (*target_it++);
      }
    }
  }

  //型を合わせるために無理矢理作り直す
  std::vector<std::shared_ptr<action::base>> re_wall{wall_.begin(), wall_.end()};

  //マーキングの処理.
  //
  //
  //
  {
    std::vector<Enemy> enemy_list;
    const auto enemy_robots = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
    for (auto it : enemy_robots) {
      const Eigen::Vector2d tmp{(it.second).x(), (it.second).y()};
      if ((ball - tmp).norm() < 300) {
        continue;
      }
      Enemy enemy_tmp{it.first, tmp, (it.second).theta(), 0.0, 0};
      enemy_list.emplace_back(enemy_tmp);
    }
    //点数の初期化
    auto point = 0;

    //ボールとの距離で点数決め
    for (auto& it : enemy_list) {
      it.valuation = (ball - it.position).norm();
    }

    //距離が近い順に昇順ソート
    std::sort(enemy_list.begin(), enemy_list.end(),
              [](const Enemy& a, const Enemy& b) { return (a.valuation < b.valuation); });

    point = enemy_list.size();
    for (auto& it : enemy_list) {
      it.score = point--;
    }

    std::sort(enemy_list.begin(), enemy_list.end(),
              [](const Enemy& a, const Enemy& b) { return (a.score > b.score); });

    auto marking_it = marking_.begin();
    auto enemy_it   = enemy_list.begin();
    while (marking_it != marking_.end() && enemy_it != enemy_list.end()) {
      (*marking_it++)->mark_robot((*enemy_it++).id);
    }
  }
  for (auto& it : marking_) {
    re_wall.push_back(it); //配列を返すためにキーパーを統合する
  }
  //ここからキーパーの処理
  //
  //
  //
  {
    Eigen::Vector2d keeper(Eigen::Vector2d::Zero());
    const auto my_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
    if (my_robots.count(keeper_id_)) {
      const auto& keeper_robot = my_robots.at(keeper_id_);
      const Eigen::Vector2d keeper_c(keeper_robot.x(), keeper_robot.y());

      switch (mode_) {
        case defense_mode::normal_mode: {
          //
          //キーパーはボールの位置によって動き方が2種類ある.
          // A:ボールが敵陣地なので多分そこまで動く必要はない
          // B:ボールが自陣地なので壁の補強をしなければ
          const auto demarcation = 2500.0;          //縄張りの大きさ
          if ((ball - goal).norm() < demarcation) { // C
            //ゴール前でディフェンスする

            {
              //ゴール前で張ってるキーパの位置
              const auto length = (goal - ball).norm(); //ゴール<->ボール
              const auto ratio = (410) / length; //全体に対してのキーパー位置の比

              keeper = (1 - ratio) * goal + ratio * ball;
            }
          } else { // B*/
            //壁のすぐ後ろで待機
            //基準点からちょっと下がったキーパの位置
            const auto length = (goal - ball).norm(); //基準点<->ボール

            const auto ratio = (1000) / length; //全体に対してのキーパー位置の比

            keeper = (1 - ratio) * goal + ratio * ball;
          }
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
          auto min_val = 0xffff;

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

          //ゴールの範囲を超えたら跳びでないようにする
          if (keeper.y() > 410) {
            keeper.y() = 410;
          } else if (keeper.y() < -410) {
            keeper.y() = -410;
          }

          //ロボットの大きさ分ずらす
          keeper.x() = goal.x() + 110.0;
          break;
        }
      }
      keeper_target_ = keeper;

      keeper_->move_to(keeper.x(), keeper.y(), ball_theta);

      re_wall.push_back(keeper_); //配列を返すためにキーパーを統合する
    } else {
      keeper_->move_to(keeper_target_.x(), keeper_target_.y(), 0.0);
    }
  }

  return re_wall; //返す
}
}
}
}
