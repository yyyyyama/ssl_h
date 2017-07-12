#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE updater_ball_test

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/filter/base.h"
#include "ai_server/model/updater/ball.h"
#include "ai_server/util/math/affine.h"

using namespace std::chrono_literals;

namespace filter = ai_server::filter;
namespace model  = ai_server::model;
namespace util   = ai_server::util;

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
    // 候補リストが空になったら何もしない (最後に選択された値になる)
    BOOST_TEST(b.x() == 10);
    BOOST_TEST(b.y() == 20);
    BOOST_TEST(b.z() == 30);
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

struct mock_filter1 : public filter::base<model::ball, filter::timing::on_updated> {
  // コンストラクタの引数に渡された値
  int arg1;
  int arg2;

  // 最後にupdateの引数に与えられた値
  model::ball v;
  std::chrono::high_resolution_clock::time_point t;

  mock_filter1(int a1, int a2) : arg1(a1), arg2(a2) {}

  model::ball update(const model::ball& value,
                     std::chrono::high_resolution_clock::time_point time) override {
    v = value;
    t = time;

    // vx, ayに適当な値をセットして返す
    model::ball r{};
    r.set_vx(value.x() * 2);
    r.set_ay(value.y() * 3);
    return r;
  }
};

BOOST_AUTO_TEST_CASE(on_updated_filter) {
  model::updater::ball bu;

  // mock_filter1を設定
  const auto fp = bu.set_filter<mock_filter1>(123, 456).lock();

  // tをhigh_resolution_clockの型に変換する関数 (長すぎ)
  auto duration_cast = [](auto t) {
    return std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(t);
  };

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
    BOOST_TEST(fp->v.x() == 1);
    BOOST_TEST(fp->v.y() == 2);
    BOOST_TEST(fp->v.z() == 3);
    BOOST_TEST(fp->t.time_since_epoch().count() == duration_cast(2s).count());

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
    BOOST_TEST(fp->v.x() == 10);
    BOOST_TEST(fp->v.y() == 20);
    BOOST_TEST(fp->v.z() == 30);
    BOOST_TEST(fp->t.time_since_epoch().count() == duration_cast(4s).count());

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
}

struct mock_filter2 : public filter::base<model::ball, filter::timing::manual> {
  using own_type = filter::base<model::ball, filter::timing::manual>;

  // コンストラクタの引数に渡された値
  int arg1;
  int arg2;

  mock_filter2(own_type::last_value_func_type lf, own_type::writer_func_type wf, int a1, int a2)
      : base(lf, wf), arg1(a1), arg2(a2) {}

  auto lv() {
    return last_value();
  }

  void wv(std::experimental::optional<model::ball> v) {
    write(v);
  }
};

BOOST_AUTO_TEST_CASE(manual_filter) {
  model::updater::ball bu;

  // mock_filter2を設定
  const auto fp = bu.set_filter<mock_filter2>(123, 456).lock();

  // set_filterの引数に与えた値がFilterのコンストラクタに正しく渡されているか
  BOOST_TEST(fp->arg1 == 123);
  BOOST_TEST(fp->arg2 == 456);

  // 初期状態はnullopt
  BOOST_TEST(!fp->lv());

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

    // last_valueから選択された値が取れる
    const auto lv = fp->lv();
    BOOST_TEST(static_cast<bool>(lv));
    BOOST_TEST(lv->x() == 1);
    BOOST_TEST(lv->y() == 2);
    BOOST_TEST(lv->z() == 3);
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
    fp->wv(std::experimental::nullopt);
    const auto b3 = bu.value();
    BOOST_TEST(b3.x() == b1.x());
    BOOST_TEST(b3.y() == b1.y());
    BOOST_TEST(b3.z() == b1.z());
  }

  {
    // 候補リストを空にするとlast_valueがnulloptを返す
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    bu.update(f);
    BOOST_TEST(!fp->lv());
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
  const auto fp2 = bu.set_filter<mock_filter2>(123, 456);
  BOOST_TEST(!fp2.expired());
  bu.clear_filter();
  BOOST_TEST(fp2.expired());

  // on_updatedなFilterが登録されている状態でmanualなFilterを登録するとon_updatedなFilterは死ぬ
  const auto fp3 = bu.set_filter<mock_filter1>(123, 456);
  const auto fp4 = bu.set_filter<mock_filter2>(123, 456);
  BOOST_TEST(fp3.expired());
  BOOST_TEST(!fp4.expired());

  // 逆も同様
  const auto fp5 = bu.set_filter<mock_filter1>(123, 456);
  BOOST_TEST(fp4.expired());
  BOOST_TEST(!fp5.expired());
}

BOOST_AUTO_TEST_SUITE_END()
