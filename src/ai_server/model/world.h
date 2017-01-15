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

  model::field field_;
  model::ball ball_;
  std::unordered_map<unsigned int, model::robot> robots_blue_;
  std::unordered_map<unsigned int, model::robot> robots_yellow_;

  /// カメラ台数分の最新のdetectionパケットを保持する
  std::unordered_map<unsigned int, ssl_protos::vision::Frame> detection_packets_;

public:
  model::field field() const;
  model::ball ball() const;
  std::unordered_map<unsigned int, model::robot> robots_blue() const;
  std::unordered_map<unsigned int, model::robot> robots_yellow() const;

  /// @brief                  内部の状態を更新する
  /// @param packet           SSL-Visionのパース済みパケット
  void update(const ssl_protos::vision::Packet& packet);

private:
  /// @brief                  detectionパケットを処理し, ボールやロボットの情報を更新する
  /// @param detection        SSL-Visionのdetectionパケット
  void process_packet(const ssl_protos::vision::Frame& detection);

  /// @brief                  geometryパケットを処理し, フィールドの情報を更新する
  /// @param geometry         SSL-Visionのgeometryパケット
  void process_packet(const ssl_protos::vision::Geometry& geometry);
};

} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_WORLD_H