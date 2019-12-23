#ifndef AI_SERVER_LOGGER_SINK_BASE_H
#define AI_SERVER_LOGGER_SINK_BASE_H

#include <string>
#include <unordered_map>
#include "ai_server/logger/log_item.h"

namespace ai_server::logger::sink {

using levels_map_type = std::unordered_map<std::string, log_level>;

class base {
  /// zone とログレベルの関係を格納するハッシュマップ
  levels_map_type levels_map_;

  /// 全 zone ("*") のログレベル
  levels_map_type::const_iterator star_level_;

public:
  base();
  base(levels_map_type levels);
  base(base&& x) noexcept;

  virtual ~base() noexcept;

  /// @brief          zone とログレベルの関係を格納するハッシュマップを取得する
  levels_map_type levels_map() const;

  /// @brief          item がこの sink で出力すべきだったら出力する
  /// @param item     出力しようとしているログ
  void check_and_do_log(const log_item& item);

protected:
  /// @brief          item を出力する
  /// @param item     出力しようとしているログ
  virtual void do_log(const log_item& item) = 0;
};

} // namespace ai_server::logger::sink

#endif // AI_SERVER_LOGGER_SINK_BASE_H
