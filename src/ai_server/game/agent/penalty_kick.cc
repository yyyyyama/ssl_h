#include "penalty_kick.h"
#include "ai_server/model/command.h"
#include "ai_server/util/math.h"

#include <cmath>
#include <boost/math/constants/constants.hpp>

namespace ai_server {
namespace game {
namespace agent {

penalty_kick::penalty_kick(const model::world& world, bool is_yellow, unsigned int kicker_id,
                           const std::vector<unsigned int>& ids)
    : base(world, is_yellow),
      mode_(penalty_kick::penalty_mode::defense),
      type_(penalty_kick::kicker_type::normal),
      start_flag_(false),
      ids_(ids),
      move_(std::make_shared<action::move>(world, is_yellow, kicker_id)),
      kick_(std::make_shared<action::kick_action>(world, is_yellow, kicker_id)),
      rush_(std::make_shared<action::rush>(world, is_yellow, kicker_id)),
      rush_move_(std::make_shared<action::move>(world, is_yellow, kicker_id)),
      kicker_id_(kicker_id),
      kick_x_(0),
      kick_y_(0),
      kick_theta_(0),
      target_x_(0),
      target_y_(0),
      keeper_id_(0),
      keeper_move_count_(0),
      prev_ball_(world_.ball()) {
  const auto goal_x = world_.field().x_max();
  const auto goal_y = 0;

  //敵キーパーを見つける
  const auto enemies = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  if (!enemies.empty()) {
    std::vector<unsigned int> enemies_id;
    for (auto& enemy : enemies) {
      enemies_id.push_back(enemy.first);
    }
    const auto keeper_it = std::min_element(
        enemies_id.cbegin(), enemies_id.cend(), [goal_x, goal_y, &enemies](auto& a, auto& b) {
          return std::hypot(enemies.at(a).x() - goal_x, enemies.at(a).y() - goal_y) <
                 std::hypot(enemies.at(b).x() - goal_x, enemies.at(b).y() - goal_y);
        });
    keeper_id_ = *keeper_it;
  }
}

penalty_kick::penalty_mode penalty_kick::mode() {
  return mode_;
}

void penalty_kick::set_mode(penalty_kick::penalty_mode mode) {
  mode_ = mode;
}

bool penalty_kick::start_flag() const {
  return start_flag_;
}

void penalty_kick::set_start_flag(bool start_flag) {
  start_flag_ = start_flag;
}

bool penalty_kick::finished() {
  // typeによって終了判定が異なる
  if (type_ == kicker_type::normal) {
    return kick_->finished();
  } else if (type_ == kicker_type::rush) {
    return rush_->finished();
  } else {
    return false;
  }
}

std::vector<std::shared_ptr<action::base>> penalty_kick::execute() {
  //////////////////////////////
  //      キッカーの処理
  /////////////////////////////

  std::vector<std::shared_ptr<action::base>> exe;
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  std::vector<unsigned int> visible_robots;
  for (auto robot : our_robots) {
    if (std::count(ids_.cbegin(), ids_.cend(), robot.first)) {
      visible_robots.push_back(robot.first);
    }
  }

  //攻撃側が指定されているとき
  if (mode_ == penalty_kick::penalty_mode::attack && our_robots.count(kicker_id_)) {
    //敵idを取得
    const auto enemies = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();

    // normalの場合のみターゲットを指定する
    // rushの場合でも一回は指定する
    if (type_ == kicker_type::normal) {
      calculate_target();
      calculate_kick_position(500);
    }
    watch_keeper();

    // normal_start前
    if (!start_flag_) {
      if (!move_->finished()) {
        //移動
        move_->move_to(kick_x_, kick_y_, kick_theta_);
        exe.push_back(move_);
      } else {
        //何もしない
        auto nop = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
        exe.push_back(nop);
        //ターゲットの計算に使うため初期化
        target_y_ = 0;
      }

      // normal_start後
    } else {
      if (!finished()) {
        //キック
        // normal
        if (type_ == kicker_type::normal) {
          kick_->kick_to(target_x_, target_y_);
          model::command::kick_flag_t kick_flag(model::command::kick_type_t::line, 10.0);
          kick_->set_kick_type(kick_flag);
          kick_->set_dribble(0);
          exe.push_back(kick_);

          // rush
        } else if (type_ == kicker_type::rush) {
          if (!rush_move_->finished()) {
            //ボールに近づく
            calculate_kick_position(200);
            rush_move_->move_to(kick_x_, kick_y_, kick_theta_);
            exe.push_back(rush_move_);
          } else {
            if (enemies.count(keeper_id_)) { //敵がいるときは離れるまで待つ
              if (std::abs(enemies.at(keeper_id_).y() - target_y_) > 400) {
                exe.push_back(rush_);
              }
            } else {
              exe.push_back(rush_);
            }
          }
        }

      } else {
        //何もしない
        auto nop = std::make_shared<action::no_operation>(world_, is_yellow_, kicker_id_);
        exe.push_back(nop);
      }
    }
  }

  //////////////////////////////
  //      キッカー以外の処理
  //////////////////////////////
  //パラメータ設定
  int count       = 2;   //何番目のロボットか判別
  double interval = 0.0; //ロボットの間隔
  double line     = 0.0; //移動位置の基準
  if (world_.ball().x() < 0) {
    line     = -2000;
    interval = 500;
  } else {
    line     = 2000;
    interval = -500;
  }

  //ちょうどいい位置に並べる
  for (auto it = visible_robots.begin(); it != visible_robots.end(); it++, count++) {
    double x = line + interval * (count / 2);
    double y = 0;
    if (count % 2) { //順番に左右に分ける
      y = world_.field().y_min() + 500;
    } else {
      y = world_.field().y_max() - 500;
    }
    auto move = std::make_shared<action::move>(world_, is_yellow_, *it);
    move->move_to(x, y, 0);
    exe.push_back(move);
  }

  return exe;
}

//ターゲットの計算
void penalty_kick::calculate_target() {
  using namespace boost::math::constants;
  const auto goal_x  = world_.field().x_max();
  const auto goal_y  = 0;
  const auto ball_y  = world_.ball().y();
  const auto enemies = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();

  //ターゲットの計算
  target_x_ = goal_x;
  if (move_->finished()) {
    //最初に決めた方向から変えない
    if (target_y_ == 0 && enemies.count(keeper_id_)) {
      target_y_ = enemies.at(keeper_id_).y() < 0 ? goal_y + 400 : goal_y - 400;
    }

    //蹴る前、PK待機位置の計算に使用
  } else {
    //最初に決めた方向から変えない
    if (target_y_ == 0) {
      target_y_ = ball_y < 0 ? goal_y + 400 : goal_y - 400;
    }
  }
}

// PK待機位置の計算
void penalty_kick::calculate_kick_position(double keep_out) {
  const auto ball_x = world_.ball().x();
  const auto ball_y = world_.ball().y();

  //パラメータ計算
  double b = 1;
  double a = (target_y_ - ball_y) / (target_x_ - ball_x);
  double c = -ball_y + (-a * ball_x);
  double l = a * a + b * b;
  double k = a * ball_x + b * ball_y + c;
  double d = l * keep_out * keep_out - k * k;

  //位置計算
  if (d > 0) {
    kick_x_ = (ball_x - a / l * k) - (b / l * std::sqrt(d));
  } else if (std::abs(d) < 0.0000000001) {
    kick_x_ = ball_x - a * k / l;
  } else {
    kick_x_ = ball_x - keep_out;
  }
  kick_y_ =
      ball_y +
      std::copysign(std::sqrt(std::pow(keep_out, 2) - std::pow(kick_x_ - ball_x, 2)), ball_y);
  kick_theta_ = std::atan2(target_y_ - ball_y, target_x_ - ball_x);
}

//敵キーパーの監視
void penalty_kick::watch_keeper() {
  const auto enemies = is_yellow_ ? world_.robots_blue() : world_.robots_yellow();
  if (!enemies.count(keeper_id_)) {
    return;
  }

  //キーパーのyを保持
  past_keeper_y_.push_back(enemies.at(keeper_id_).y());

  //キーパーがゴールを守ってる
  auto minmax = std::minmax_element(past_keeper_y_.begin(), past_keeper_y_.end());
  if (*minmax.second < 600 && *minmax.first > -600 && keeper_move_count_ < 5) {
    if (std::fabs(*minmax.second - *minmax.first) > 800) { //過去のyの最大値、最小値の差
      keeper_move_count_++;
      past_keeper_y_.clear();
    }
  }

  if (keeper_move_count_ >= 3) { //往復回数が3を超えると、rushに変更
    type_ = kicker_type::rush;
  }
}
}
}
}