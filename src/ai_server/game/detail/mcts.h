#ifndef AI_SERVER_GAME_DETAIL_MCTS_H
#define AI_SERVER_GAME_DETAIL_MCTS_H

#include <forward_list>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>
#include <Eigen/Core>
#include <nbla/context.hpp>
#include <nbla/init.hpp>
#include <nbla_utils/nnp.hpp>

#include "ai_server/game/nnabla.h"
#include "ai_server/model/world.h"

namespace ai_server::game::detail::mcts {

// 状態
struct state {
  // ボールの位置
  Eigen::Vector2d ball_pos;
  // ボールを持つロボット
  unsigned int chaser;
  // ロボット
  model::world::robots_list our_robots;
  model::world::robots_list ene_robots;
  // 到達までの時間
  double t = 0.0;
};

// MCTS用のノード
struct node {
  std::recursive_mutex mtx;

  // 状態
  struct state state;
  // このノードへの到達確率
  double p;
  // 価値
  double w;
  // 最大報酬
  double max_v;
  // 訪問回数
  int n;
  // 子ノード
  std::forward_list<node> child_nodes;

  node(const struct state& s, double p = 1.0) : state(s), p(p), w(0.0), max_v(0.0), n(0) {}
};

// MCTSによってノードの評価を行う
class evaluator {
  std::vector<std::unique_ptr<nbla::utils::nnp::Nnp>> nnp_ptrs_;
  // nnpファイルに紐付けられたkey
  const std::string probability_key_ = "probability";
  // nnpファイルを扱うデータ型
  const std::string probability_data_type_ = "float";

public:
  /// @brief コンストラクタ
  /// @param nnabla 使用するnnpの情報が格納されたgame::nnabla
  evaluator(const game::nnabla& nnabla,
            unsigned int num = std::thread::hardware_concurrency() - 1);

  /// @brief MCTSを実行する
  /// @param field フィールド情報
  /// @param our_goal_pos 自陣ゴール
  /// @param ene_goal_pos 敵陣ゴール
  /// @param root_node 開始時の状態を表すノード
  void execute(const model::field& field, const Eigen::Vector2d& our_goal_pos,
               const Eigen::Vector2d& ene_goal_pos, node& root_node);
};

} // namespace ai_server::game::detail::mcts
#endif
