#ifndef AI_SERVER_LOGGER_FORMATTER_H
#define AI_SERVER_LOGGER_FORMATTER_H

#include <string>
#include <string_view>

namespace ai_server::logger {

struct log_item;

/// @brief  \p item を \p fmt に従って文字列を生成する
/// \p fmt には以下のようなパターンが利用できる
///   - {elapsed}:      経過時間
///   - {level}:        ログレベル
///   - {level_simple}: ログレベル ([+], [!], [-], ...)
///   - {message}:      ログメッセージ
///   - {thread_id}:    スレッドID
///   - {time}:         日時
///   - {zone}:         zone の名前
std::string format(std::string_view fmt, const log_item& item);

} // namespace ai_server::logger

#endif // AI_SERVER_LOGGER_FORMATTER_H
