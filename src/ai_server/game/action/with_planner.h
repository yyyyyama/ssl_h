#ifndef AI_SERVER_GAME_ACTION_WITH_PLANNER_H
#define AI_SERVER_GAME_ACTION_WITH_PLANNER_H

#include <memory>
#include <type_traits>

#include "ai_server/planner/obstacle_list.h"
#include "base.h"

namespace ai_server::planner {
class base;
}

namespace ai_server::game::action {

/// action を wrap し、目標値を planner に掛けたものを出力する
class with_planner : public action::base {
  std::shared_ptr<action::base> action_;
  std::unique_ptr<planner::base> planner_;
  planner::obstacle_list obstacles_;

public:
  template <class Action,
            std::enable_if_t<std::is_base_of_v<action::base, Action> &&
                                 !std::is_base_of_v<action::self_planning_base, Action> &&
                                 !std::is_same_v<action::base, Action> &&
                                 !std::is_same_v<with_planner, Action>,
                             std::nullptr_t> = nullptr>
  with_planner(std::shared_ptr<Action>& action, std::unique_ptr<planner::base> planner,
               const planner::obstacle_list& obstacles)
      : base{*action}, action_{action}, planner_{std::move(planner)}, obstacles_{obstacles} {}

  bool finished() const override;

  model::command execute() override;
};

} // namespace ai_server::game::action

#endif // ifndef AI_SERVER_GAME_ACTION_WITH_PLANNER_H
