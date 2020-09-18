#define BOOST_TEST_DYN_LINK

#include <string>
#include <boost/test/unit_test.hpp>

#include "ai_server/filter/base.h"

using namespace std::string_literals;
namespace filter = ai_server::filter;

BOOST_AUTO_TEST_SUITE(filter_base)

// 新しい値を受け取ったときに更新されるfilter.
// filter::base<任意の型, filter::timing::same>を継承し,
// update()メンバ関数をoverrideすれば良い.
class test_filter1 : public filter::base<std::string, filter::timing::same> {
  std::string suffix_;

public:
  // 必要があれば任意のコンストラクタを追加する
  test_filter1(const std::string& suffix) : suffix_(suffix) {}

  // filterの更新処理
  std::optional<std::string> update(std::optional<std::string> value,
                                    std::chrono::system_clock::time_point) override {
    // 何らかの処理を行い, 結果を返す
    if (value.has_value()) {
      return *value + suffix_;
    } else {
      return std::nullopt;
    }
  }
};

BOOST_AUTO_TEST_CASE(same) {
  std::chrono::system_clock::time_point t{};

  test_filter1 f{"er"};

  BOOST_TEST(f.update("C++", t).value() == "C++er"s);
  BOOST_TEST(f.update("Haskell", t).value() == "Haskeller"s);
}

// 値の更新を任意のタイミングで行うfilter.
// filter::base<任意の型, filter::timing::manual>を継承し,
// 更新を行うpublicメンバ関数を実装すれば良い.
class test_filter2 : public filter::base<std::string, filter::timing::manual> {
  std::string prefix_;
  std::optional<std::string> value_;

public:
  // コンストラクタで値を受け取る必要がなければ
  // using filter::base<std::string, filter::timing::manual>::base;
  // コンストラクタで値を受け取る必要がある場合は次のようにする
  test_filter2(std::recursive_mutex& mutex, test_filter2::writer_func_type wf,
               const std::string& prefix)
      : base(mutex, wf), prefix_(prefix) {}

  // 観測値を受け取るメンバ関数
  void set_raw_value(std::optional<std::string> value,
                     std::chrono::system_clock::time_point) override {
    std::unique_lock lock{mutex()};
    value_ = std::move(value);
  }

  // 更新を行うメンバ関数
  // 名前は適当で良い
  void add_prefix() {
    std::unique_lock lock{mutex()};
    if (value_) {
      // 値の更新はwrite()を使う
      write(prefix_ + *value_);
    } else {
      // std::nulloptで更新すれば値が存在しないものとして処理される
      write(std::nullopt);
    }
  }
};

BOOST_AUTO_TEST_CASE(manual) {
  std::recursive_mutex mutex{};

  // 更新対象のデータ
  std::optional<std::string> target{};
  // 値を更新するための関数
  auto writer = [&target](std::optional<std::string> new_value) { target = new_value; };

  test_filter2 f{mutex, writer, "すごい"};

  // filterを初期化するだけでは値は変化しない
  BOOST_TEST(!target.has_value());

  // nulloptでも問題ない
  BOOST_CHECK_NO_THROW(f.add_prefix());

  // filterの状態を更新すると値が変化する
  f.set_raw_value("Haskell", {});
  f.add_prefix();
  BOOST_TEST(*target == "すごいHaskell");

  // 関数が登録されていなくても例外で落ちない
  test_filter2 bad_filter{mutex, {}, ""};
  BOOST_CHECK_NO_THROW(bad_filter.add_prefix());
}

BOOST_AUTO_TEST_SUITE_END()
