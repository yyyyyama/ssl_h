#include "kick_off_waiter.h"

#include <cmath>

namespace ai_server {
namespace game {
namespace agent {

kick_off_waiter::kick_off_waiter(const model::world& world, bool is_yellow,
                                 const std::vector<unsigned int>& ids)
    : base(world, is_yellow), ids_(ids), mode_(kickoff_mode::attack) {}

std::vector<std::shared_ptr<action::base>> kick_off_waiter::execute() {
  std::vector<std::shared_ptr<action::base>> exe;
  const auto robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();

  //視認可能なロボットをids_から抽出する
  std::vector<unsigned int> visible_robots;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_robots),
               [&robots](auto id) { return robots.count(id); });
  auto tmp_ids  = visible_robots;
  auto ids_size = visible_robots.size();

  //攻撃側
  if (mode_ == kickoff_mode::attack) {
    int count_a     = 2; //何番目のロボットか判別
    int count_b     = 2;
    double interval = 500.0;  //ロボットの間隔
    double x_line   = -200.0; // xは-200固定

    for (auto it = tmp_ids.begin(); it != tmp_ids.end(); it++) {
      auto move = std::make_shared<action::move>(world_, is_yellow_, *it);
      auto a    = std::hypot((x_line - robots.at(*it).x()),
                          (world_.field().y_min() + interval * count_a - robots.at(*it).y()));
      auto b    = std::hypot((x_line - robots.at(*it).x()),
                          (world_.field().y_max() - interval * count_b - robots.at(*it).y()));
      if (a < b) {
        move->move_to(x_line, world_.field().y_min() + interval * count_a, 0);
        count_a++;
      } else {
        move->move_to(x_line, world_.field().y_max() - interval * count_b, 0);
        count_b++;
      }
      exe.push_back(move);
    }

    //守備側
  } else {
    const auto enemies = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
    const auto ball    = world_.ball();
    std::vector<unsigned int> enemies_id;
    for (auto& enemy : enemies) {
      enemies_id.push_back(enemy.first);
    }

    //敵キッカーをマーク候補から除外
    const auto ball_near_it = std::min_element(
        enemies_id.cbegin(), enemies_id.cend(), [&enemies, &ball](auto& a, auto& b) {
          return std::hypot(enemies.at(a).x() - ball.x(), enemies.at(a).y() - ball.y()) <
                 std::hypot(enemies.at(b).x() - ball.x(), enemies.at(b).y() - ball.y());
        });
    //敵キッカーの座標を取得し,マーキング候補から除外
    double enemy_theta = 0;
    double enemy_x     = 0;
    double enemy_y     = 0;
    if (ball_near_it != enemies_id.end()) {
      enemy_theta = enemies.at(*ball_near_it).theta();
      enemy_x     = enemies.at(*ball_near_it).x();
      enemy_y     = enemies.at(*ball_near_it).y();
      enemies_id.erase(ball_near_it);
    }

    //常に敵キッカーの目の前にいるような位置取りにする
    double r = 750; //センターサークルに入らないような半径
    double x = r * std::cos(enemy_theta) < -20 ? 700 * std::cos(enemy_theta)
                                               : -700 * std::cos(enemy_theta);
    double y     = r * std::sin(enemy_theta);
    double theta = std::atan2(enemy_y - y, enemy_x - x);
    const auto it =
        std::min_element(tmp_ids.cbegin(), tmp_ids.cend(), [&robots, x, y](auto& a, auto& b) {
          return std::hypot(robots.at(a).x() - x, robots.at(a).y() - y) <
                 std::hypot(robots.at(b).x() - x, robots.at(b).y() - y);
        });
    if (it != tmp_ids.end()) {
      auto move = std::make_shared<action::move>(world_, is_yellow_, *it);
      tmp_ids.erase(it);
      move->move_to(x, y, theta);
      exe.push_back(move);
    }

    //敵をセンターラインに近い順にソート
    std::sort(enemies_id.begin(), enemies_id.end(),
              [&](auto& a, auto& b) { return enemies.at(a).x() < enemies.at(b).x(); });

    //敵をセンターラインに近い順にマーク
    for (int i = 0; i < ids_size - 1; i++) {
      if (tmp_ids.empty()) {
        return exe;
      }

      //マーク位置の決定
      if ((enemies_id.begin() + i) == enemies_id.end()) {
        return exe;
      }
      double y     = enemies.at(enemies_id[i]).y();
      double x     = -700;
      double theta = 0;

      //マークするロボットの決定
      const auto it =
          std::min_element(tmp_ids.cbegin(), tmp_ids.cend(), [&robots, x, y](auto& a, auto& b) {
            return std::hypot(robots.at(a).x() - x, robots.at(a).y() - y) <
                   std::hypot(robots.at(b).x() - x, robots.at(b).y() - y);
          });
      if (it != tmp_ids.end()) {
        auto move = std::make_shared<action::move>(world_, is_yellow_, *it);
        move      = std::make_shared<action::move>(world_, is_yellow_, *it);
        move->move_to(x, y, theta);
        exe.push_back(move);
        tmp_ids.erase(it);
      }
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
} // namespace agent
} // namespace game
} // namespace ai_server
