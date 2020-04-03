#include "impl/rrt_star.h"
#include "rrt_star.h"

namespace ai_server::planner {

rrt_star::rrt_star(const model::world& world)
    : world_(world), node_count_(10), max_branch_length_(300.0) {}

void rrt_star::set_node_count(int count) {
  node_count_ = count;
}

void rrt_star::set_max_branch_length(double length) {
  max_branch_length_ = length;
}

base::planner_type rrt_star::planner() {
  return [this](const position_t& start, const position_t& goal, const obstacle_list& obs) {
    impl::rrt_star rrt{world_};
    rrt.set_max_pos(max_pos_);
    rrt.set_min_pos(min_pos_);
    rrt.set_margin(margin_);
    rrt.set_node_count(node_count_);
    rrt.set_max_branch_length(max_branch_length_);
    return rrt.execute(start, goal, obs);
  };
}
} // namespace ai_server::planner
