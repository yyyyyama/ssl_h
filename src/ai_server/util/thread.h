#ifndef AI_SERVER_UTIL_THREAD_H
#define AI_SERVER_UTIL_THREAD_H

#include <string_view>
#include <thread>

// int pthread_setname_np(pthread_t, const char*) が呼び出せるか確認
// - macOS でない
//   - macOS の int pthread_setname_np(const char*) は現在のスレッドの名前しか設定できない
// - 標準ライブラリ内部で GNU C Library 2.12+ を使っている
// - <pthread.h> がある

#if __has_include(<pthread.h>)
extern "C" {
#include <pthread.h>
}
#endif

#if !defined(__APPLE__) && defined(__GLIBC__)
#if __GLIBC_PREREQ(2, 12)
#define AI_SERVER_HAS_PTHREAD_SETNAME_NP 1
#else
#define AI_SERVER_HAS_PTHREAD_SETNAME_NP 0
#endif
#else
#define AI_SERVER_HAS_PTHREAD_SETNAME_NP 0
#endif

namespace ai_server::util {

/// @brief         スレッド名を設定する (可能な場合)
/// @param thread  対象のスレッド
/// @param name    名前
/// @return        スレッド名の設定に成功したか
static inline bool set_thread_name([[maybe_unused]] std::thread& thread,
                                   [[maybe_unused]] std::string_view name) {
#if AI_SERVER_HAS_PTHREAD_SETNAME_NP
  // std::thread::native_handle_type が pthread_t であるか確認
  if constexpr (std::is_same_v<std::thread::native_handle_type, ::pthread_t>) {
    auto nh = thread.native_handle();
    return ::pthread_setname_np(nh, name.data()) == 0;
  }
#endif

  return false;
}

} // namespace ai_server::util

#undef AI_SERVER_HAS_PTHREAD_SETNAME_NP

#endif // AI_SERVER_UTIL_THREAD_H
