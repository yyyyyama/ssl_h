#ifndef AI_SERVER_MODEL_DETAIL_CONFIDENCE_COMPARATOR_H
#define AI_SERVER_MODEL_DETAIL_CONFIDENCE_COMPARATOR_H

namespace ai_server {
namespace model {
namespace detail {

static auto confidence_comparator = [](const auto& a, const auto& b) {
  return a.confidence() > b.confidence();
};

} // namespace detail
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_DETAIL_CONFIDENCE_COMPARATOR_H
