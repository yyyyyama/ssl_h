#include <cmath>
#include <tuple>
#include "ai_server/util/math/to_vector.h"
#include "kick_off.h"

namespace ai_server {
namespace game {
namespace agent {

kick_off::kick_off(const model::world& world, bool is_yellow, unsigned int kicker_id,
                   const std::vector<unsigned int>& waiter)
    : base(world, is_yellow),
      kicker_id_(kicker_id),
      waiter_(waiter),
      start_flag_(false),
      evacuate_finished_(false),
      move_finished_(false),
      kick_finished_(false),
      receive_finished_(false),
      move_(std::make_shared<action::move>(world_, is_yellow_, kicker_id_)),
      kick_(std::make_shared<action::kick>(world_, is_yellow_, kicker_id_)),
      get_ball_(std::make_shared<action::get_ball>(world_, is_yellow_, kicker_id_)) {}

kick_off::kick_off(const model::world& world, bool is_yellow, unsigned int kicker_id)
    : base(world, is_yellow),
      kicker_id_(kicker_id),
      start_flag_(false),
      evacuate_finished_(false),
      move_finished_(false),
      kick_finished_(false),
      receive_finished_(true),
      move_(std::make_shared<action::move>(world_, is_yellow_, kicker_id_)),
      kick_(std::make_shared<action::kick>(world_, is_yellow_, kicker_id_)),
      get_ball_(std::make_shared<action::get_ball>(world_, is_yellow_, kicker_id_)) {}

void kick_off::set_start_flag(bool start_flag) {
  start_flag_ = start_flag;
}

bool kick_off::start_flag() const {
  return start_flag_;
}

bool kick_off::finished() const {
  return move_finished_ && kick_finished_ && receive_finished_;
}

std::vector<std::shared_ptr<action::base>> kick_off::execute() {
  std::vector<std::shared_ptr<action::base>> actions;
  const auto ball = world_.ball();

  //見えないときの処理
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if (!our_robots.count(kicker_id_)) {
    return actions;
  }
  //待機ロボットで見えているものをレシーバー候補にする
  std::vector<unsigned int> visible_waiter;
  for (auto tmp : waiter_) {
    if (our_robots.count(tmp)) {
      visible_waiter.push_back(tmp);
    }
  }

  if (kick_finished_) {
    //ボールを蹴り終わった時
    auto no_op = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
    actions.push_back(no_op);

    if (!visible_waiter.empty() || std::hypot(ball.vx(), ball.vy()) < 100.0) {
      receive_finished_ = receive_->finished();
    } else {
      //待機ロボットがいないならパス判定はなし
      //ボール速度がある程度遅かったら終了
      receive_finished_ = true;
    }
  } else {
    if (move_finished_ && start_flag_) {
      // StartGameが指定され、所定の位置に移動済みの時
      auto kick_type = std::make_tuple(model::command::kick_type_t::line, 40.0);
      auto kick_mode = action::kick::mode::goal;
      double kick_to_x;
      double kick_to_y;

      if (!visible_waiter.empty()) {
        // 待機ロボットの中で一番近いロボットをレシーバーにする
        const auto nearest =
            std::min_element(visible_waiter.cbegin(), visible_waiter.cend(),
                             [&ball, &our_robots](auto& a, auto& b) {
                               const auto first  = util::math::position(our_robots.at(a));
                               const auto second = util::math::position(our_robots.at(b));
                               return (first - util::math::position(ball)).norm() <
                                      (second - util::math::position(ball)).norm();
                             });
        // 蹴った後は変えない
        if (!kick_finished_) {
          receiver_id_ = *nearest;
        }
        kick_to_x = our_robots.at(receiver_id_).x();
        kick_to_y = our_robots.at(receiver_id_).y();
      } else {
        //待機ロボットがいないなら適当に放り投げる
        kick_to_x = world_.field().x_max();
        kick_to_y = 0.0;
        kick_type = std::make_tuple(model::command::kick_type_t::backspin, 120.0);
      }
      kick_->kick_to(kick_to_x, kick_to_y);
      kick_->set_kick_type(kick_type);
      kick_->set_mode(kick_mode);
      kick_finished_ = kick_->finished();
      actions.push_back(kick_);
    } else {
      // StartGameが指定されていない、または所定の位置に移動していない時
      const auto this_robot_team = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
      const auto& this_robot     = this_robot_team.at(kicker_id_);
      ball_goal_theta_         = std::atan2(0.0 - ball.y(), world_.field().x_max() - ball.x());
      theta_min_               = std::asin(1); //移動中にロボットがボールに近づけないようにするための値
      const double hypot_allow = std::hypot(
          10, 10); //ボールとの衝突回避に用いられる、指定位置と実際の位置のズレの許容値
      const double keep_out_r = 500.0; //キックオフ時の立ち入り禁止区域の半径
      const double robot_r    = 90.0;  //ロボットの半径

      //ボールとゴールを結んだライン上で、ボールから半径keep_out_r+ロボット半径離れたところを指定
      move_to_x_     = ball.x() - (keep_out_r + robot_r) * std::cos(ball_goal_theta_);
      move_to_y_     = ball.y() - (keep_out_r + robot_r) * std::sin(ball_goal_theta_);
      move_to_theta_ = ball_goal_theta_;

      //-- ロボットがボールにぶつからないようにするための処理 --

      move_to_robot_theta_ =
          std::atan2(this_robot.y() - move_to_y_, this_robot.x() - move_to_x_);

      if (std::hypot(this_robot.x() - move_to_x_, this_robot.y() - move_to_y_) >
              keep_out_r + robot_r + hypot_allow &&
          this_robot.x() > move_to_x_ &&
          (std::abs(move_to_robot_theta_ - ball_goal_theta_) < evacuate_finished_
               ? theta_min_ - std::atan2(keep_out_r, 30.0)
               : theta_min_)) {
        //ロボットがボールにぶつかる可能性がある時

        if (move_to_robot_theta_ < ball_goal_theta_) {
          //移動経路が短いほうを選択させる
          theta_min_ = -1.0 * theta_min_;
        }

        //ボールをよけるための位置を指定
        move_to_x_ += (keep_out_r + robot_r) * std::cos(ball_goal_theta_ + theta_min_);
        move_to_y_ += (keep_out_r + robot_r) * std::sin(ball_goal_theta_ + theta_min_);
        evacuate_finished_ = move_->finished();

      } else {
        move_finished_ =
            std::hypot(this_robot.x() - move_to_x_, this_robot.y() - move_to_y_) < 250.0;
      }
      move_->move_to(move_to_x_, move_to_y_, move_to_theta_);
      actions.push_back(move_);
    }
  }

  // waiterの処理
  if (!visible_waiter.empty()) {
    if (!kick_finished_) {
      //蹴られるまではkickoff waiterと同じ処理
      int count_a           = 2; //何番目のロボットか判別
      int count_b           = 2;
      const double interval = 500.0;  //ロボットの間隔
      const double x_line   = -200.0; // xは-200固定
      double y;
      std::vector<unsigned int> move_y;

      for (auto it : visible_waiter) {
        auto move = std::make_shared<action::move>(world_, is_yellow_, it);
        const auto a =
            std::hypot((x_line - our_robots.at(it).x()),
                       (world_.field().y_min() + interval * count_a - our_robots.at(it).y()));
        const auto b =
            std::hypot((x_line - our_robots.at(it).x()),
                       (world_.field().y_max() - interval * count_b - our_robots.at(it).y()));
        if (a < b) {
          y = world_.field().y_min() + interval * count_a - 200;
          count_a++;
        } else {
          y = world_.field().y_max() - interval * count_b;
          count_b++;
        }

        // 重なるときは移動しない
        if (std::find_if(move_y.cbegin(), move_y.cend(), [&](const auto m) {
              return std::abs(m - (world_.field().y_min() + interval * count_a)) < 500.0;
            }) != move_y.cend()) {
          continue;
        }
        move->move_to(
            x_line, y,
            std::atan2(ball.y() - our_robots.at(it).y(), ball.x() - our_robots.at(it).x()));
        move_y.push_back(y);
        actions.push_back(move);
      }
    } else {
      for (auto tmp : visible_waiter) {
        if (tmp != receiver_id_) {
          //レシーバー以外は何もしない
          auto no_op = std::make_shared<action::no_operation>(world_, is_yellow_, tmp);
          actions.push_back(no_op);
        } else {
          //レシーバーだけはreceive actionをする
          receive_ = std::make_shared<action::receive>(world_, is_yellow_, tmp);
          receive_->set_passer(kicker_id_);
          receive_->set_dribble(9);
          receive_->set_kick_type(std::make_tuple(model::command::kick_type_t::none, 0.0));
          actions.push_back(receive_);
        }
      }
    }
  }
  return actions;
}

} // namespace agent
} // namespace game
} // namespace ai_server
