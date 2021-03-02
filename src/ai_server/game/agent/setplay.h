#ifndef AI_SERVER_GAME_AGENT_SETPLAY_H
#define AI_SERVER_GAME_AGENT_SETPLAY_H

#include <chrono>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include "base.h"
#include "ai_server/game/action/get_ball.h"
#include "ai_server/model/world.h"
#include "ai_server/game/action/kick.h"
#include "ai_server/game/action/receive.h"

namespace ai_server {
namespace game {
namespace agent {

class setplay : public base {
public:
  // finished:動作が終わった状態, setup:指定位置へ移動, pass:パスを蹴る, receive:パスを受け取る,
  // shoot:シュートする
  enum class state { finished, setup, pass, receive, shoot };
  enum class pos { near, mid, far };
  setplay(context& ctx, unsigned int kicker_id, const std::vector<unsigned int>& receiver_id);

  std::vector<unsigned int> free_robots() const;

  std::vector<std::shared_ptr<action::base>> execute() override;
  void set_mode(int mode_num);
  void set_direct(bool is_direct);

  bool finished() const;

private:
  pos pos_;
  state state_ = state::setup;
  unsigned int kicker_id_;
  unsigned int shooter_id_;
  const std::vector<unsigned int> receiver_ids_;
  std::chrono::steady_clock::time_point start_point_;
  Eigen::Vector2d prev_ball_vel_;
  std::shared_ptr<action::kick> kick_;
  std::shared_ptr<action::get_ball> get_ball_;
  std::shared_ptr<action::receive> receive_;
  Eigen::Vector2d passpos_;
  Eigen::Vector2d shoot_pos;
  std::vector<Eigen::Vector2d> positions_;
  bool neflag        = false;
  bool receive_flag_ = false;
  int shooter_num_   = 0;
  int change_count_  = 0;
  double ballysign;
  bool is_direct_ = false;
  bool try_direct = false;

  std::vector<unsigned int> free_robots_;

  int mode_;

  int chose_location(std::vector<Eigen::Vector2d> targets,
                     model::world::robots_list enemy_robots, int dist = -1);
  double vectorangle(Eigen::Vector2d vec) const;
};
} // namespace agent
} // namespace game
} // namespace ai_server
#endif
