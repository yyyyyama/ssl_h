#include <cmath>
#include <tuple>
#include "kick_off.h"

namespace ai_server {
namespace game {
namespace agent {

kick_off::kick_off(const model::world& world, bool is_yellow, unsigned int kicker_id)
    : base(world, is_yellow),
      kicker_id_(kicker_id),
      move_(std::make_shared<action::move>(world_, is_yellow_, kicker_id_)),
      kick_(std::make_shared<action::kick_action>(world_, is_yellow_, kicker_id_)) {
  start_flag_    = false;
  move_finished_ = false;
  kick_finished_ = false;
}

void kick_off::set_start_flag(bool start_flag) {
  start_flag_ = start_flag;
}

bool kick_off::start_flag() const {
  return start_flag_;
}

std::vector<std::shared_ptr<action::base>> kick_off::execute() {
  const double robot_r    = 90.0;  //ロボットの半径
  const double keep_out_r = 500.0; //キックオフ時の立ち入り禁止区域の半径
  theta_min_              = std::asin((50 + robot_r) /
                         keep_out_r); //移動中にロボットがボールに近づけないようにするための値

  std::vector<std::shared_ptr<action::base>> actions;

  if (kick_finished_) {
    //ボールを蹴り終わった時
    auto no_op = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
    actions.push_back(no_op);

  } else {
    if (move_finished_ && start_flag_) {
      // StartGameが指定され、所定の位置に移動済みの時
      auto kick_type = std::make_tuple(model::command::kick_type_t::backspin, 60.0);
      auto kick_mode = action::kick_action::mode::goal;

      kick_->kick_to(world_.field().x_max(), 0.0);
      kick_->set_kick_type(kick_type);
      kick_->set_mode(kick_mode);
      kick_finished_ = kick_->finished();
      actions.push_back(kick_);

    } else {
      // StartGameが指定されていない、または所定の位置に移動していない時
      const auto ball            = world_.ball();
      const auto this_robot_team = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
      const auto& this_robot     = this_robot_team.at(kicker_id_);
      ball_goal_theta_           = atan(0.0 - ball.y() / (world_.field().x_max() - ball.x()));

      //ボールとゴールを結んだライン上で、ボールから半径keep_out_r+ロボット半径離れたところを指定
      move_to_x_     = ball.x() - (keep_out_r + robot_r) * std::cos(ball_goal_theta_);
      move_to_y_     = ball.y() - (keep_out_r + robot_r) * std::sin(ball_goal_theta_);
      move_to_theta_ = ball_goal_theta_;

      //-- ロボットがボールにぶつからないようにするための処理 --

      move_to_robot_theta_ =
          std::atan((this_robot.y() - move_to_y_) / (this_robot.x() - move_to_x_));

      if (std::sqrt(std::pow(this_robot.x() - move_to_x_, 2) +
                    std::pow(this_robot.y() - move_to_y_, 2)) >
              (keep_out_r + robot_r) * std::cos(move_to_robot_theta_ - ball_goal_theta_) &&
          this_robot.x() > move_to_x_ &&
          std::abs(move_to_robot_theta_ - ball_goal_theta_) < theta_min_) {
        //ロボットがボールにぶつかる可能性がある時

        if (move_to_robot_theta_ < ball_goal_theta_) {
          //移動経路が短いほうを選択させる
          theta_min_ = -1.0 * theta_min_;
        }

        //ボールをよけるための位置を指定
        move_to_x_ += (keep_out_r + robot_r) * std::cos(theta_min_) *
                      std::cos(ball_goal_theta_ + theta_min_);
        move_to_y_ += (keep_out_r + robot_r) * std::cos(theta_min_) *
                      std::sin(ball_goal_theta_ + theta_min_);
      }

      move_->move_to(move_to_x_, move_to_y_, move_to_theta_);
      move_finished_ = move_->finished();
      actions.push_back(move_);
    }
  }

  return actions;
}

} // namespace agent
} // namespace agent
} // napespace ai_server
