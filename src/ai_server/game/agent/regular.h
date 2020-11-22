#ifndef AI_SERVER_GAME_AGENT_REGULAR_H
#define AI_SERVER_GAME_AGENT_REGULAR_H

#include <queue>
#include <unordered_map>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include "ai_server/game/action/get_ball.h"
#include "ai_server/game/action/marking.h"
#include "ai_server/game/action/move.h"
#include "ai_server/game/action/receive.h"
#include "base.h"

namespace ai_server::game::agent {

class regular : public base {
public:
  // normal : 通常のモード, no_mark : 敵のマークをしない
  enum mark_option { normal, no_mark };

  regular(context& ctx, const std::vector<unsigned int>& ids);
  regular(context& ctx, const std::vector<unsigned int>& ids, unsigned int default_chaser);
  bool has_chaser() const;
  unsigned int chaser_id() const;
  void customize_marking(mark_option option);
  // ボールを追いかけるか否か（全てのロボットをマーキングにする時はfalse）
  void use_chaser(bool use_chaser);
  std::vector<std::shared_ptr<action::base>> execute() override;

private:
  // 構造体：IDと重要度
  struct id_importance {
    unsigned int id;
    double importance;

    bool operator<(const id_importance& next) const;
  };

  // エリアを示す構造体(x1,y1)〜(x2,y2)までの四角形
  struct area {
    double x1;
    double y1;
    double x2;
    double y2;
    double score; // 評価する際に用いる
  };

  // 位置
  using point = boost::geometry::model::d2::point_xy<double>;
  // 使うロボットのID
  const std::vector<unsigned int> ids_;
  // ボールを追いかけてもいいか否か
  bool can_chase_;
  //　ボールを追いかけるか
  bool has_chaser_;
  // chaserを初期化する必要があるか
  bool need_reset_chaser_;
  // マークの形態
  mark_option mark_option_;
  // ボールを持ったロボットが蹴る先を保持
  point target_;
  // ボールを持っているロボットのID
  unsigned int chaser_id_;
  //　敵のリスト、重要度順
  std::priority_queue<id_importance> enemy_list_;
  // 1枚目のマーキングが対象とする敵リスト
  std::unordered_map<unsigned int, unsigned int> marked_list_;
  // 1枚目のマーキング
  std::unordered_map<unsigned int, std::shared_ptr<action::marking>> first_marking_;
  // 2枚目のマーキング
  std::unordered_map<unsigned int, std::shared_ptr<action::marking>> second_marking_;
  // move
  std::unordered_map<unsigned int, std::shared_ptr<action::move>> move_;
  // receive
  std::unordered_map<unsigned int, std::shared_ptr<action::receive>> receive_;
  // マーキング割り当ての際に余ったロボットID
  std::vector<unsigned int> follower_ids_;
  // moveロボットの移動先
  std::unordered_map<unsigned int, point> reserved_points_;
  // moveロボットの移動先を少しの間保持
  std::unordered_map<unsigned int, point> reserved_points_old_;
  // get_ball
  std::shared_ptr<action::get_ball> get_ball_;

  // chase id_の候補を取得
  unsigned int select_chaser() const;
  // 1枚目のマーキングの再構成
  void update_first_marking();
  // 2枚目のマーキングの再構成
  void update_second_mariking();
  //　余ったロボットへの割当て
  void update_recievers();
  // 敵リストの作成
  std::priority_queue<id_importance> make_enemy_list() const;
  // パス経路を生成してポイントを返す
  point select_target() const;
  // ターゲットに最も近いロボットID
  std::vector<unsigned int>::const_iterator nearest_id(const std::vector<unsigned int>& can_ids,
                                                       double target_x, double target_y) const;
  // 空いているエリアを返す
  area empty_area(const area& area, unsigned int my_id,
                  const std::vector<unsigned int>& our_ids) const;
  // 指定位置がエリア上にあるかどうか
  bool in_area(double x, double y, const regular::area& area) const;
};

} // namespace ai_server::game::agent

#endif
