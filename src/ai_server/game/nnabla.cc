#include <algorithm>

#include <boost/algorithm/string/predicate.hpp>

#include "nnabla.h"

using namespace std::string_literals;

namespace ai_server::game {

nnabla::nnabla(const std::vector<std::string>& backend, const std::string& device_id,
               const std::unordered_map<std::string, nnp_file_type>& nnp_files)
    : backend_{backend}, device_id_{device_id}, nnp_files_{nnp_files} {}

std::pair<nbla::Context, std::string> nnabla::nnp(const std::string& key,
                                                  const std::string& data_type,
                                                  bool cached) const {
  // TODO: key が存在しなかったときの扱いを検討する
  // 常に存在することを仮定してそのまま、返り値を std::optional に包む、など
  const auto& [path, use_cpu] = nnp_files_.at(key);

  const auto backend = [this, use_cpu = use_cpu, &data_type]() -> std::vector<std::string> {
    if (use_cpu) {
      return {"cpu:" + data_type};
    } else {
      std::vector<std::string> backend{};
      std::transform(backend_.cbegin(), backend_.cend(), std::back_inserter(backend),
                     [&data_type](const auto& b) { return b + ":" + data_type; });
      return backend;
    }
  }();

  const auto array_class =
      (std::any_of(backend.cbegin(), backend.cend(),
                   [](const auto& b) {
                     using boost::algorithm::starts_with;
                     return starts_with(b, "cuda:") || starts_with(b, "cudnn:");
                   })
           ? "Cuda"s
           : "Cpu"s) +
      (cached ? "CachedArray"s : "Array"s);

  return {nbla::Context{backend, array_class, device_id_}, path};
}

} // namespace ai_server::game
