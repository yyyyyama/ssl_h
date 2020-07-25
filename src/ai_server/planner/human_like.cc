#include "ai_server/util/math/to_vector.h"
#include "impl/human_like.h"
#include "human_like.h"

namespace ai_server::planner {

void human_like::set_direction_count(int count) {
  direction_count_ = count;
}

void human_like::set_max_exit_length(double length) {
  max_exit_length_ = length;
}

void human_like::set_max_length(double length) {
  max_length_ = length;
}

void human_like::set_min_length(double length) {
  min_length_ = length;
}

void human_like::set_step_length(double length) {
  step_length_ = length;
}

base::planner_type human_like::planner() {
  return [this](const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                const obstacle_list& obs) {
    // 障害物
    const auto obstacles = obs.to_tree();
    // 移動可能領域
    impl::box_type area{min_pos_, max_pos_};
    // スタートからゴールまでのベクトル
    const Eigen::Vector2d sg = goal - start;

    // 長さを調整
    const double l_max = std::min(max_length_, sg.norm());
    const double l_min = std::min(min_length_, l_max);

    // 方向と長さのリスト
    const auto lengths = impl::make_length_list(l_min, l_max, step_length_);
    const auto dirs    = impl::make_directions(
        sg.norm() > 0.0 ? sg.normalized() : Eigen::Vector2d::UnitX(), direction_count_);

    // Human-Likeによる探索結果
    const auto plan_result = impl::planned_position(start, dirs, lengths, obstacles, area);

    // 最終的な結果
    Eigen::Vector2d result;
    if (plan_result.has_value()) {
      result = plan_result.value();
    } else {
      // 脱出
      const auto exit_lengths = impl::make_length_list(l_min, max_exit_length_, step_length_);
      const auto exit_p       = impl::exit_position(start, dirs, exit_lengths, obstacles, area);

      result = exit_p.value_or(
          // 最低でも min_length_ は進ませる
          impl::default_position(start, sg.normalized(), min_length_, area));
    }

    return std::make_pair(result, (result - start).norm());
  };
}
} // namespace ai_server::planner
