#define BOOST_TEST_DYN_LINK

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/filter/base.h"
#include "ai_server/model/updater/ball.h"
#include "ai_server/util/math/affine.h"

using namespace std::chrono_literals;

namespace filter = ai_server::filter;
namespace model  = ai_server::model;
namespace util   = ai_server::util;

// tをsystem_clock::durationに変換する関数 (長すぎ)
auto dc = [](auto t) {
  return std::chrono::duration_cast<std::chrono::system_clock::duration>(t);
};

BOOST_AUTO_TEST_SUITE(updater_ball)

BOOST_AUTO_TEST_CASE(normal) {
  model::updater::ball bu;

  {
    // デフォルトコンストラクが呼ばれたときに, 内部の値がちゃんと初期化されているか
    const auto b1 = bu.value();
    const auto b2 = model::ball{};
    BOOST_TEST(b1.x() == b2.x());
    BOOST_TEST(b1.y() == b2.y());
    BOOST_TEST(b1.z() == b2.z());
    BOOST_TEST(b1.is_lost() == b2.is_lost());
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);

    auto b1 = f.add_balls();
    b1->set_x(1);
    b1->set_y(2);
    b1->set_z(3);
    b1->set_confidence(90.0);

    auto b2 = f.add_balls();
    b2->set_x(4);
    b2->set_y(5);
    b2->set_z(6);
    b2->set_confidence(92.0);

    bu.update(f);
  }

  {
    const auto b = bu.value();
    // confidenceの高い方の値が選択される
    BOOST_TEST(b.x() == 4);
    BOOST_TEST(b.y() == 5);
    BOOST_TEST(b.z() == 6);
    BOOST_TEST(b.is_lost() == false);
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(1);

    auto b1 = f.add_balls();
    b1->set_x(10);
    b1->set_y(20);
    b1->set_z(30);
    b1->set_confidence(94.0);

    bu.update(f);
  }

  {
    const auto b = bu.value();
    // 違うカメラでよりconfidenceの高い方の値が検出されたらそちらが選択される
    BOOST_TEST(b.x() == 10);
    BOOST_TEST(b.y() == 20);
    BOOST_TEST(b.z() == 30);
    BOOST_TEST(b.is_lost() == false);
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(2);

    auto b1 = f.add_balls();
    b1->set_x(100);
    b1->set_y(200);
    b1->set_z(300);
    b1->set_confidence(88.0);

    bu.update(f);
  }

  {
    const auto b = bu.value();
    // 違うカメラでconfidenceの低い値が検出されても値は変化しない
    BOOST_TEST(b.x() == 10);
    BOOST_TEST(b.y() == 20);
    BOOST_TEST(b.z() == 30);
    BOOST_TEST(b.is_lost() == false);
  }

  {
    ssl_protos::vision::Frame f;
    // 現在選択されている値が検出されたcam1の値を消す
    f.set_camera_id(1);
    bu.update(f);
  }

  {
    const auto b = bu.value();
    // cam0で検出されたball{4, 5, 6}が選ばれるが, 古い値では更新されない
    BOOST_TEST(b.x() == 10);
    BOOST_TEST(b.y() == 20);
    BOOST_TEST(b.z() == 30);
    BOOST_TEST(b.is_lost() == false);
  }

  {
    ssl_protos::vision::Frame f;
    // 候補リストを空にする
    f.set_camera_id(0);
    bu.update(f);
    f.set_camera_id(2);
    bu.update(f);
  }

  {
    const auto b = bu.value();
    // 候補リストが空になったら最後に選択された値になる
    // is_lost が true になる
    BOOST_TEST(b.x() == 10);
    BOOST_TEST(b.y() == 20);
    BOOST_TEST(b.z() == 30);
    BOOST_TEST(b.is_lost() == true);
  }
}

BOOST_AUTO_TEST_CASE(transformation, *boost::unit_test::tolerance(0.0000001)) {
  using namespace boost::math::double_constants;

  model::updater::ball bu;
  // 90度回転, x軸方向に10, y軸方向に20平行移動
  bu.set_transformation_matrix(util::math::make_transformation_matrix(10.0, 20.0, half_pi));

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);

    auto b1 = f.add_balls();
    b1->set_x(100);
    b1->set_y(200);
    b1->set_z(300);
    b1->set_confidence(90.0);

    bu.update(f);
  }

  {
    const auto b = bu.value();
    BOOST_TEST(b.x() == -190);
    BOOST_TEST(b.y() == 120.0);
    BOOST_TEST(b.z() == 300);
  }
}

struct mock_filter1 : public filter::base<model::ball, filter::timing::same> {
  // コンストラクタの引数に渡された値
  int arg1;
  int arg2;

  // 最後にupdateの引数に与えられた値
  std::optional<model::ball> v;
  std::chrono::system_clock::time_point t;

  mock_filter1(int a1, int a2) : arg1(a1), arg2(a2) {}

  std::optional<model::ball> update(std::optional<model::ball> value,
                                    std::chrono::system_clock::time_point time) override {
    v = value;
    t = time;

    if (v.has_value()) {
      // vx, ayに適当な値をセットして返す
      model::ball r{};
      r.set_vx(value->x() * 2);
      r.set_ay(value->y() * 3);
      return r;
    } else {
      return std::nullopt;
    }
  }
};

BOOST_AUTO_TEST_CASE(filter_same) {
  model::updater::ball bu;

  // mock_filter1を設定
  const auto fp = bu.set_filter<mock_filter1>(123, 456).lock();

  // set_filterの引数に与えた値がFilterのコンストラクタに正しく渡されているか
  BOOST_TEST(fp->arg1 == 123);
  BOOST_TEST(fp->arg2 == 456);

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto b1 = f.add_balls();
    b1->set_x(1);
    b1->set_y(2);
    b1->set_z(3);
    b1->set_confidence(90.0);

    bu.update(f);
  }

  {
    // updateの引数に値が正しく渡されているか
    BOOST_TEST(fp->v->x() == 1);
    BOOST_TEST(fp->v->y() == 2);
    BOOST_TEST(fp->v->z() == 3);
    BOOST_TEST(fp->t.time_since_epoch().count() == dc(2s).count());

    // vx, ayが選択された値の2倍, 3倍になっている
    const auto b = bu.value();
    BOOST_TEST(b.vx() == 2);
    BOOST_TEST(b.ay() == 6);
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(1);
    f.set_t_capture(4.0);

    auto b1 = f.add_balls();
    b1->set_x(10);
    b1->set_y(20);
    b1->set_z(30);
    b1->set_confidence(92.0);

    bu.update(f);
  }

  {
    // updateの引数に値が正しく渡されているか
    BOOST_TEST(fp->v->x() == 10);
    BOOST_TEST(fp->v->y() == 20);
    BOOST_TEST(fp->v->z() == 30);
    BOOST_TEST(fp->t.time_since_epoch().count() == dc(4s).count());

    // vx, ayが選択された値の2倍, 3倍になっている
    const auto b = bu.value();
    BOOST_TEST(b.vx() == 20);
    BOOST_TEST(b.ay() == 60);
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(8.0);

    auto b1 = f.add_balls();
    b1->set_x(100);
    b1->set_y(200);
    b1->set_z(300);
    b1->set_confidence(90.0);

    bu.update(f);
  }

  {
    // 最新でない値が選択された場合は変化しない
    const auto b = bu.value();
    BOOST_TEST(b.vx() == 20);
    BOOST_TEST(b.ay() == 60);
  }

  {
    ssl_protos::vision::Frame f1;
    f1.set_camera_id(1);
    f1.set_t_capture(10.0);

    bu.update(f1);

    ssl_protos::vision::Frame f2;
    f2.set_camera_id(0);
    f2.set_t_capture(10.0);

    bu.update(f2);
  }

  {
    // 対象がロストしたらnulloptが渡されている
    BOOST_TEST(!fp->v.has_value());
    BOOST_TEST(fp->t.time_since_epoch().count() == dc(10s).count());
  }
}

struct mock_filter2 : public filter::base<model::ball, filter::timing::same> {
  std::optional<model::ball> update(std::optional<model::ball> value,
                                    std::chrono::system_clock::time_point) override {
    // 引数に値が渡された時にnullopt, そうでない時に値を返すFilter
    if (value.has_value()) {
      return std::nullopt;
    } else {
      return model::ball{123, 456, 789};
    }
  }
};

BOOST_AUTO_TEST_CASE(filter_same2) {
  model::updater::ball bu;

  // mock_filter2を設定
  const auto fp = bu.set_filter<mock_filter2>().lock();

  {
    // 初期状態
    const auto b = bu.value();
    BOOST_TEST(b.x() == 0);
    BOOST_TEST(b.y() == 0);
    BOOST_TEST(b.z() == 0);
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto b1 = f.add_balls();
    b1->set_x(1);
    b1->set_y(2);
    b1->set_z(3);
    b1->set_confidence(90.0);

    bu.update(f);

    const auto b = bu.value();
    BOOST_TEST(b.x() == 0);
    BOOST_TEST(b.y() == 0);
    BOOST_TEST(b.z() == 0);
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(4.0);

    bu.update(f);

    // 空で更新したら値がセットされている
    const auto b = bu.value();
    BOOST_TEST(b.x() == 123);
    BOOST_TEST(b.y() == 456);
    BOOST_TEST(b.z() == 789);
  }
}

struct mock_filter3 : public filter::base<model::ball, filter::timing::manual> {
  // コンストラクタの引数に渡された値
  int arg1;
  int arg2;

  // set_raw_value で渡された値
  std::optional<model::ball> value;
  std::chrono::system_clock::time_point time;

  mock_filter3(std::recursive_mutex& mutex, mock_filter3::writer_func_type wf, int a1, int a2)
      : base(mutex, wf), arg1(a1), arg2(a2) {}

  void set_raw_value(std::optional<model::ball> v,
                     std::chrono::system_clock::time_point t) override {
    std::unique_lock lock{mutex()};
    value = v;
    time  = t;
  }

  void wv(std::optional<model::ball> v) {
    std::unique_lock lock{mutex()};
    write(v);
  }
};

BOOST_AUTO_TEST_CASE(manual_filter) {
  model::updater::ball bu;

  // mock_filter3を設定
  const auto fp = bu.set_filter<mock_filter3>(123, 456).lock();

  // set_filterの引数に与えた値がFilterのコンストラクタに正しく渡されているか
  BOOST_TEST(fp->arg1 == 123);
  BOOST_TEST(fp->arg2 == 456);

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto b1 = f.add_balls();
    b1->set_x(1);
    b1->set_y(2);
    b1->set_z(3);
    b1->set_confidence(90.0);

    bu.update(f);
  }

  {
    // updateが呼ばれただけでは値は更新されない
    const auto b1 = bu.value();
    const auto b2 = model::ball{};
    BOOST_TEST(b1.x() == b2.x());
    BOOST_TEST(b1.y() == b2.y());
    BOOST_TEST(b1.z() == b2.z());

    // set_raw_value から観測値が渡されている
    const auto lv = fp->value;
    BOOST_TEST(lv->x() == 1);
    BOOST_TEST(lv->y() == 2);
    BOOST_TEST(lv->z() == 3);

    const auto lt = fp->time;
    BOOST_TEST(lt.time_since_epoch().count() == dc(2s).count());
  }

  {
    // writeで値が更新される
    const auto b1 = model::ball{4, 5, 6};
    fp->wv(b1);

    const auto b2 = bu.value();
    BOOST_TEST(b2.x() == b1.x());
    BOOST_TEST(b2.y() == b1.y());
    BOOST_TEST(b2.z() == b1.z());

    // nulloptで更新しても値は変化しない
    fp->wv(std::nullopt);
    const auto b3 = bu.value();
    BOOST_TEST(b3.x() == b1.x());
    BOOST_TEST(b3.y() == b1.y());
    BOOST_TEST(b3.z() == b1.z());
  }

  {
    // 候補リストを空にするとset_raw_valueにnulloptが渡されている
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(4.0);
    bu.update(f);

    const auto lv = fp->value;
    BOOST_TEST(!lv.has_value());
    const auto lt = fp->time;
    BOOST_TEST(lt.time_since_epoch().count() == dc(4s).count());
  }
}

BOOST_AUTO_TEST_CASE(clear_filter) {
  model::updater::ball bu;

  // clear_filterを呼ぶと登録したFilterが死んでいる
  const auto fp1 = bu.set_filter<mock_filter1>(123, 456);
  BOOST_TEST(!fp1.expired());
  bu.clear_filter();
  BOOST_TEST(fp1.expired());

  // manualなFilterも同様
  const auto fp2 = bu.set_filter<mock_filter3>(123, 456);
  BOOST_TEST(!fp2.expired());
  bu.clear_filter();
  BOOST_TEST(fp2.expired());

  // sameなFilterが登録されている状態でmanualなFilterを登録するとsameなFilterは死ぬ
  const auto fp3 = bu.set_filter<mock_filter1>(123, 456);
  const auto fp4 = bu.set_filter<mock_filter3>(123, 456);
  BOOST_TEST(fp3.expired());
  BOOST_TEST(!fp4.expired());

  // 逆も同様
  const auto fp5 = bu.set_filter<mock_filter1>(123, 456);
  BOOST_TEST(fp4.expired());
  BOOST_TEST(!fp5.expired());
}

// set_raw_value() で write() を呼ぶ filter
struct mock_filter4 : public filter::base<model::ball, filter::timing::manual> {
  mock_filter4(std::recursive_mutex& mutex, mock_filter4::writer_func_type wf)
      : base(mutex, wf) {}

  void set_raw_value(std::optional<model::ball> v,
                     std::chrono::system_clock::time_point) override {
    std::unique_lock lock{mutex()};
    write(v);
  }

  void wv(std::optional<model::ball> v) {
    std::unique_lock lock{mutex()};
    write(v);
  }
};

BOOST_AUTO_TEST_CASE(manual_filter2, *boost::unit_test::tolerance(0.0000001)) {
  model::updater::ball bu;
  const auto fp = bu.set_filter<mock_filter4>().lock();

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto b1 = f.add_balls();
    b1->set_x(1);
    b1->set_y(2);
    b1->set_z(3);
    b1->set_confidence(90.0);

    bu.update(f);
  }

  {
    // ボールの値が更新されている
    const auto b = bu.value();
    BOOST_TEST(b.x() == 1);
    BOOST_TEST(b.y() == 2);
    BOOST_TEST(b.z() == 3);
  }

  {
    // writeで値が更新される
    fp->wv(model::ball{4, 5, 6});

    const auto b = bu.value();
    BOOST_TEST(b.x() == 4);
    BOOST_TEST(b.y() == 5);
    BOOST_TEST(b.z() == 6);
  }
}

BOOST_AUTO_TEST_SUITE_END()
