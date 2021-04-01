#ifndef AI_SERVER_MODEL_MOTION_BASE_H
#define AI_SERVER_MODEL_MOTION_BASE_H

#include <memory>
#include <string>

namespace ai_server::model::motion {

class base {
public:
  base(const std::string& motion_id);

  virtual ~base() = default;

  /// @brief 呼び出されたループでのロボットの命令を取得する
  virtual std::tuple<double, double, double> execute() = 0;

  std::string motion_id();

protected:
  std::string motion_id_;
};

} // namespace ai_server::model::motion

#endif