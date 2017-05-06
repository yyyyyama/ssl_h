#ifndef AI_SERVER_MODEL_WORLD_H
#define AI_SERVER_MODEL_WORLD_H

#include <mutex>
#include <string>
#include <unordered_map>

#include "ball.h"
#include "field.h"
#include "robot.h"

#include "ssl-protos/vision/wrapperpacket.pb.h"

namespace ai_server {
namespace model {

/// @class   world
/// @brief   SSL-Visionからデータを内部で扱いやすい形式に処理する
class world {
  mutable std::mutex mutex_;

public:
  using robots_list = std::unordered_map<unsigned int, model::robot>;

  world();

  // コピーコンストラクタ
  world(const world& others);
  world& operator=(const world& others);

  model::field field() const;
  model::ball ball() const;
  robots_list robots_blue() const;
  robots_list robots_yellow() const;

  template <class T>
  void set_field(T&& field) {
    std::lock_guard<std::mutex> lock(mutex_);
    field_ = std::forward<T>(field);
  }

  template <class T>
  void set_ball(T&& ball) {
    std::lock_guard<std::mutex> lock(mutex_);
    ball_ = std::forward<T>(ball);
  }

  template <class T>
  void set_robots_blue(T&& robots_blue) {
    std::lock_guard<std::mutex> lock(mutex_);
    robots_blue_ = std::forward<T>(robots_blue);
  }

  template <class T>
  void set_robots_yellow(T&& robots_yellow) {
    std::lock_guard<std::mutex> lock(mutex_);
    robots_yellow_ = std::forward<T>(robots_yellow);
  }

private:
  model::field field_;
  model::ball ball_;
  robots_list robots_blue_;
  robots_list robots_yellow_;
};

} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_WORLD_H
