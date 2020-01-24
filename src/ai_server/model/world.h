#ifndef AI_SERVER_MODEL_WORLD_H
#define AI_SERVER_MODEL_WORLD_H

#include <string>
#include <unordered_map>

#include "ball.h"
#include "field.h"
#include "robot.h"
#include "team_color.h"

namespace ai_server {
namespace model {

/// @class   world
/// @brief   SSL-Visionからのデータを表現するクラス
class world {
public:
  using robots_list = std::unordered_map<unsigned int, model::robot>;

  world() = default;

  world(model::field&& field, model::ball&& ball, robots_list&& robots_blue,
        robots_list&& robots_yellow);

  model::field field() const;
  model::ball ball() const;
  robots_list robots_blue() const;
  robots_list robots_yellow() const;

  template <class T>
  void set_field(T&& field) {
    field_ = std::forward<T>(field);
  }

  template <class T>
  void set_ball(T&& ball) {
    ball_ = std::forward<T>(ball);
  }

  template <class T>
  void set_robots_blue(T&& robots_blue) {
    robots_blue_ = std::forward<T>(robots_blue);
  }

  template <class T>
  void set_robots_yellow(T&& robots_yellow) {
    robots_yellow_ = std::forward<T>(robots_yellow);
  }

private:
  model::field field_;
  model::ball ball_;
  robots_list robots_blue_;
  robots_list robots_yellow_;
};

// @brief \p w から \p color のロボットの情報を取得する
inline auto our_robots(const world& w, team_color color) {
  switch (color) {
    case team_color::blue:
      return w.robots_blue();
    case team_color::yellow:
      return w.robots_yellow();
  }
}

// @brief \p w から \p color の敵ロボットの情報を取得する
inline auto enemy_robots(const world& w, team_color color) {
  switch (color) {
    case team_color::blue:
      return w.robots_yellow();
    case team_color::yellow:
      return w.robots_blue();
  }
}

} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_WORLD_H
