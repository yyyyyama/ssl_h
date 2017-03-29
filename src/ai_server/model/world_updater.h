#ifndef AI_SERVER_MODEL_WORLD_UPDATER_H
#define AI_SERVER_MODEL_WORLD_UPDATER_H

#include <unordered_map>

#include "world.h"

#include "ssl-protos/vision/wrapperpacket.pb.h"

namespace ai_server {
namespace model {

class world_updater {
  model::world world_;

  /// カメラ台数分の最新のdetectionパケットを保持する
  std::unordered_map<unsigned int, ssl_protos::vision::Frame> detection_packets_;

public:
  /// @brief                  WorldModelを取得する
  const model::world& world_model() const;

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

#endif // AI_SERVER_MODEL_WORLD_UPDATER_H
