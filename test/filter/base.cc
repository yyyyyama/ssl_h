#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE filter_base_test

#include <string>
#include <boost/test/unit_test.hpp>

#include "ai_server/filter/base.h"

using namespace std::string_literals;
namespace filter = ai_server::filter;

BOOST_AUTO_TEST_SUITE(filter_base)

// 新しい値を受け取ったときに更新されるfilter.
// filter::base<任意の型, filter::timing::on_updated>を継承し,
// update()メンバ関数をoverrideすれば良い.
class test_filter1 : public filter::base<std::string, filter::timing::on_updated> {
  std::string suffix_;

public:
  // 必要があれば任意のコンストラクタを追加する
  test_filter1(const std::string& suffix) : suffix_(suffix) {}

  // filterの更新処理
  std::string update(const std::string& value,
                     std::chrono::high_resolution_clock::time_point) override {
    // 何らかの処理を行い, 結果を返す
    return value + suffix_;
  }
};

BOOST_AUTO_TEST_CASE(on_updated) {
  std::chrono::high_resolution_clock::time_point t{};

  test_filter1 f{"er"};

  BOOST_TEST(f.update("C++", t) == "C++er"s);
  BOOST_TEST(f.update("Haskell", t) == "Haskeller"s);
}

// 値の更新を任意のタイミングで行うfilter.
// filter::base<任意の型, filter::timing::manual>を継承し,
// 更新を行うpublicメンバ関数を実装すれば良い.
class test_filter2 : public filter::base<std::string, filter::timing::manual> {
  // 型が長いので型エイリアスを作っておくと良いかも
  using own_type = filter::base<std::string, filter::timing::manual>;

  std::string prefix_;

public:
  // コンストラクタで値を受け取る必要がなければ
  // using filter::base<std::string, filter::timing::manual>::base;
  // コンストラクタで値を受け取る必要がある場合は次のようにする
  test_filter2(own_type::last_value_func_type lf, own_type::writer_func_type wf,
               const std::string& prefix)
      : base(lf, wf), prefix_(prefix) {}

  // 更新を行うメンバ関数
  // 名前は適当で良い
  void add_prefix() {
    // 対象データの取得にはlast_value()を使う
    const auto value = last_value();
    if (value) {
      // 値の更新はwrite()を使う
      write(prefix_ + *value);
    } else {
      // std::experimental::nulloptで更新すれば値が存在しないものとして処理される
      write(std::experimental::nullopt);
    }
  }
};

BOOST_AUTO_TEST_CASE(manual) {
  // 更新対象のデータ
  std::experimental::optional<std::string> value{"Haskell"};

  // 値を取得するための関数
  auto f1 = [&value] { return value; };
  // 値を更新するための関数
  auto f2 = [&value](std::experimental::optional<std::string> new_value) { value = new_value; };

  test_filter2 f{f1, f2, "すごい"};

  // filterを初期化するだけでは値は変化しない
  BOOST_TEST(*value == "Haskell");

  // filterの状態を更新すると値が変化する
  f.add_prefix();
  BOOST_TEST(*value == "すごいHaskell");
  f.add_prefix();
  BOOST_TEST(*value == "すごいすごいHaskell");
  f.add_prefix();
  BOOST_TEST(*value == "すごいすごいすごいHaskell");

  // nulloptでも問題ない
  value = std::experimental::nullopt;
  BOOST_CHECK_NO_THROW(f.add_prefix());

  // 関数が登録されていなくても例外で落ちない
  test_filter2 bad_filter{{}, {}, ""};
  BOOST_CHECK_NO_THROW(bad_filter.add_prefix());
}

BOOST_AUTO_TEST_SUITE_END()
