#ifndef AI_SERVER_MODEL_MOTION_BASE_H
#define AI_SERVER_MODEL_MOTION_BASE_H

#include <memory>
#include <string>

namespace ai_server::model::motion {

class base {
public:
  base(const std::string& name);

  std::string name();

private:
  std::string name_;
};

} // namespace ai_server::model::motion

#endif