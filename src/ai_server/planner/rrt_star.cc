#include <cmath>
#include <random>
#include "ai_server/planner/rrt_star.h"

namespace ai_server {
namespace planner {
rrt_star::rrt_star(const model::world& world) : world_(world) {
  // ペナルティエリアを障害物指定(正方形の外接円で近似)
  default_obstacles_.push_back(object{position_t{world_.field().x_max(), 0.0, 0.0},
                                      static_cast<double>(world_.field().penalty_length())});
  default_obstacles_.push_back(object{position_t{world_.field().x_min(), 0.0, 0.0},
                                      static_cast<double>(world_.field().penalty_length())});
}

bool rrt_star::obstructed(const position_t start, const position_t goal, const double margin) {
  // 2点が完全に一致していたら
  if (start.x == goal.x && start.y == goal.y) {
    return false;
  }

  double a = goal.y - start.y;
  double b = -(goal.x - start.x);
  double c = -a * start.x - b * start.y;

  bool obstructed = false;
  for (auto it = all_obstacles_.begin(); it < all_obstacles_.end(); ++it) {
    // 障害物と直線の距離
    double dist =
        std::abs(a * (*it).position_.x + b * (*it).position_.y + c) / std::hypot(a, b);
    // 直線の方向ベクトル
    double vx = (goal.x - start.x) / std::hypot(start.x - goal.x, start.y - goal.y);
    double vy = (goal.y - start.y) / std::hypot(start.x - goal.x, start.y - goal.y);
    // 障害物判定のための内点
    double px1 = start.x + vx * (margin + (*it).r_);
    double py1 = start.y + vy * (margin + (*it).r_);
    double px2 = goal.x + vx * (margin + (*it).r_);
    double py2 = goal.y + vy * (margin + (*it).r_);

    // 線に近く，p1,p2の点の四角形内に障害物があったら
    if (dist < margin + (*it).r_ && (*it).position_.x + (*it).r_ > std::min(px1, px2) &&
        (*it).position_.x - (*it).r_ < std::max(px1, px2) &&
        (*it).position_.y + (*it).r_ > std::min(py1, py2) &&
        (*it).position_.y - (*it).r_ < std::max(py1, py2)) {
      obstructed = true;
      break;
    }
  }
  return obstructed;
}

bool rrt_star::in_penalty(const position_t point, const double margin) {
  if (std::abs(point.x) > (world_.field().x_max() - world_.field().penalty_length() - margin) &&
      std::abs(point.y) < (world_.field().penalty_width() / 2.0 - margin)) {
    return true;
  } else {
    return false;
  }
}

bool rrt_star::out_field(const position_t point, const double margin) {
  if (std::abs(point.x) > world_.field().x_max() + margin ||
      std::abs(point.y) > world_.field().y_max() + margin) {
    return true;
  } else {
    return false;
  }
}

void rrt_star::set_obstacles(const std::vector<object>& obstacles) {
  all_obstacles_.clear();
  all_obstacles_ = obstacles;
  std::copy(default_obstacles_.begin(), default_obstacles_.end(),
            std::back_inserter(all_obstacles_));
}

void rrt_star::search(const position_t start, const position_t goal, const position_t max_pos,
                      const position_t min_pos, const int search_num,
                      const double max_branch_length, const double margin) {
  struct object near_obstacle;
  bool in_avoid_range = false;
  for (auto&& it : all_obstacles_) {
    if (std::hypot(start.x - it.position_.x, start.y - it.position_.y) < it.r_) {
      near_obstacle  = it;
      in_avoid_range = true;
    }
  }
  if (in_avoid_range) { // 避ける範囲内に開始地点が存在,特別処理
    double angle =
        std::atan2(start.y - near_obstacle.position_.y, start.x - near_obstacle.position_.x);
    target_.x     = near_obstacle.position_.x + near_obstacle.r_ * std::cos(angle);
    target_.y     = near_obstacle.position_.y + near_obstacle.r_ * std::sin(angle);
    target_.theta = 0.0;
  } else if (in_penalty(start, margin) ||
             out_field(start, 200.0)) { // penalty内かfield外，特別処理
    target_ = position_t{0.0, 0.0, 0.0};
  } else { // RRT*
    tree_.clear();
    // 初期ノード追加
    tree_.push_back(std::make_shared<struct node>(node{start, 0.0, {}}));

    int search_count = 0;
    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_real_distribution<> rand_x(min_pos.x - margin, max_pos.x + margin);
    std::uniform_real_distribution<> rand_y(min_pos.y - margin, max_pos.y + margin);

    double min_dist_to_goal = 10000; // 目標位置までの最短距離(初期値は大きく取る)

    while (search_count < search_num) {
      search_count++;
      position_t search_pos; // 次探索点
      bool on_obstacle;      // 障害物上は探索しない
      double min_dist                       = 100000.0;
      std::shared_ptr<struct node> min_node = nullptr;
      position_t min_new_node;
      position_t next_node;
      do {
        do {
          on_obstacle = false;
          if (priority_points_.size() == 0) {
            search_pos.x = rand_x(mt);
            search_pos.y = rand_y(mt);
          } else {
            search_pos = priority_points_.front();
            priority_points_.pop();
          }
          for (auto&& it : all_obstacles_) {
            if (std::hypot(search_pos.x - it.position_.x, search_pos.y - it.position_.y) <
                (it.r_ + margin)) {
              on_obstacle = true;
            }
          }
        } while (on_obstacle);
        // ツリー中の全ノードから次探索点までの距離を算出，最近傍で障害物のないものを選ぶ
        for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
          // 距離算出
          double dist = std::hypot(search_pos.x - tree_[i]->position_.x,
                                   search_pos.y - tree_[i]->position_.y);
          next_node.x = std::min(dist, max_branch_length) *
                            (search_pos.x - tree_[i]->position_.x) / dist +
                        tree_[i]->position_.x;
          next_node.y = std::min(dist, max_branch_length) *
                            (search_pos.y - tree_[i]->position_.y) / dist +
                        tree_[i]->position_.y;

          // 距離比較
          if (min_dist > dist) {
            // 障害物検査
            if (!(obstructed(search_pos, next_node))) {
              min_dist     = dist;
              min_node     = tree_[i];
              min_new_node = next_node;
            }
          }
        }
      } while (min_node == nullptr);

      // 伸ばせるノードを見つけていたらtree追加
      if (min_node != nullptr) {
        std::shared_ptr<struct node> new_node = std::make_shared<struct node>(node{
            min_new_node, min_node->cost_ + std::min(min_dist, max_branch_length), min_node});

        tree_.push_back(new_node);
        // 新ノードからゴールまでの距離を計算
        double dist_bet_goal_to_new_node =
            std::hypot(goal.x - new_node->position_.x, goal.y - new_node->position_.y);
        if (min_dist_to_goal > dist_bet_goal_to_new_node) {
          min_dist_to_goal = dist_bet_goal_to_new_node;
          nearest_node_    = new_node;
        }
        // これまでの道と新しいノードからの道を比較してコスト低ならノード再接続
        for (auto it = tree_.begin(); it != tree_.end(); ++it) {
          // 追加した物自身だったらやめる
          if (*it != new_node) {
            // コストの比較
            double rewired_cost =
                new_node->cost_ + std::hypot(new_node->position_.x - (*it)->position_.x,
                                             new_node->position_.y - (*it)->position_.y);
            if ((*it)->cost_ > rewired_cost) {
              // 障害物検査
              search_pos = (*it)->position_;
              next_node  = new_node->position_;
              if (!(obstructed(search_pos, next_node))) {
                (*it)->cost_   = rewired_cost;
                (*it)->parent_ = new_node;
              }
            }
          }
        }
      }
    }
    auto node = nearest_node_;
    std::queue<position_t>().swap(priority_points_);
    priority_points_.push(node->position_);
    // 障害物を挟まないノードをショートカットして
    // スムースをかける
    while (!node->parent_.expired() && !node->parent_.lock()->parent_.expired() &&
           node->parent_.lock()->parent_.lock() != nullptr) {
      if (!obstructed(node->position_, node->parent_.lock()->parent_.lock()->position_,
                      100.0)) {
        node->parent_ = node->parent_.lock()->parent_.lock();
      } else {
        node = node->parent_.lock();
        priority_points_.push(node->position_);
        target_ = node->position_;
      }
    }
    target_ = node->position_;
  }
  target_.theta = goal.theta;
}

} // namespace planner
} // namespace ai_server
