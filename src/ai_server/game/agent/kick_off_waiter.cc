#include "kick_off_waiter.h"

#include <cmath>

namespace ai_server {
namespace game {
namespace agent {

kick_off_waiter::kick_off_waiter(const model::world& world, bool is_yellow,
                                 const std::vector<unsigned int>& ids)
    : base(world, is_yellow), ids_(ids) {}

std::vector<std::shared_ptr<action::base>> kick_off_waiter::execute() {
  std::vector<std::shared_ptr<action::base>> exe;
  std::shared_ptr<action::move> move;
  const auto& robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  auto tmp_ids       = ids_;

  //攻撃側
  if (mode_ == kickoff_mode::attack) {
    int count       = 2;      //何番目のロボットか判別
    double interval = 500.0;  //ロボットの間隔
    double x_line   = -200.0; // xは-200固定

    for (auto it = tmp_ids.begin(); it != tmp_ids.end(); it++, count++) {
      move = std::make_shared<action::move>(world_, is_yellow_, *it);
      if (count % 2) {
        move->move_to(x_line, world_.field().y_min() + interval * (count / 2), 0);
      } else {
        move->move_to(x_line, world_.field().y_max() - interval * (count / 2), 0);
      }
      exe.push_back(move);
    }

    //守備側
  } else {
    const auto& enemys = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
    const auto& ball   = world_.ball();
    std::vector<std::tuple<double, double>> distination;
    std::tuple<double, double> distination_tmp;
    std::vector<unsigned int> enemys_id;
    for (auto& enemy : enemys) {
      enemys_id.push_back(enemy.first);
    }

    const auto ball_near_it = std::min_element(
        enemys_id.cbegin(), enemys_id.cend(), [&enemys, &ball](auto& a, auto& b) {
          return std::hypot(enemys.at(a).x() - ball.x(), enemys.at(a).y() - ball.y()) <
                 std::hypot(enemys.at(b).x() - ball.x(), enemys.at(b).y() - ball.y());
        });
    if (ball_near_it != enemys_id.end()) {
      enemys_id.erase(ball_near_it);
    }

    //一体はボールの前に固定
    double x     = -700;
    double y     = 0;
    double theta = 0;
    const auto it =
        std::min_element(tmp_ids.cbegin(), tmp_ids.cend(), [&robots, x, y](auto& a, auto& b) {
          return std::hypot(robots.at(a).x() - x, robots.at(a).y() - y) <
                 std::hypot(robots.at(b).x() - x, robots.at(b).y() - y);
        });
    move = std::make_shared<action::move>(world_, is_yellow_, *it);
    tmp_ids.erase(it);
    move->move_to(x, y, theta);
    exe.push_back(move);

    //他は敵のxが小さい順にマーク
    for (int i = 0; i < ids_.size() - 1; i++) {
      if (tmp_ids.empty()) {
        return exe;
      }

      //マーク位置の決定
      const auto enemy_it = std::min_element(
          enemys_id.cbegin(), enemys_id.cend(),
          [&enemys](auto& a, auto& b) { return enemys.at(a).x() < enemys.at(b).x(); });
      double y     = enemys.at(*enemy_it).y();
      double x     = -700;
      double theta = 0;
      enemys_id.erase(enemy_it);

      //マークするロボットの決定
      const auto it =
          std::min_element(tmp_ids.cbegin(), tmp_ids.cend(), [&robots, x, y](auto& a, auto& b) {
            return std::hypot(robots.at(a).x() - x, robots.at(a).y() - y) <
                   std::hypot(robots.at(b).x() - x, robots.at(b).y() - y);
          });
      move = std::make_shared<action::move>(world_, is_yellow_, *it);
      move->move_to(x, y, theta);
      exe.push_back(move);
      tmp_ids.erase(it);
    }
  }

  return exe;
}

void kick_off_waiter::set_mode(kickoff_mode mode) {
  mode_ = mode;
}

kick_off_waiter::kickoff_mode kick_off_waiter::mode() {
  return mode_;
}
}
}
}
