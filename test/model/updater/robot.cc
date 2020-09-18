#define BOOST_TEST_DYN_LINK

#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/filter/base.h"
#include "ai_server/model/updater/robot.h"
#include "ai_server/util/math/affine.h"

using namespace std::chrono_literals;

namespace filter = ai_server::filter;
namespace model  = ai_server::model;
namespace util   = ai_server::util;

// 角度をいい感じに作りたかったので
constexpr double rad(double deg) {
  using namespace boost::math::double_constants;
  return deg * pi / 180;
}

// tをsystem_clock::durationに変換する関数 (長すぎ)
auto dc = [](auto t) {
  return std::chrono::duration_cast<std::chrono::system_clock::duration>(t);
};

BOOST_AUTO_TEST_SUITE(updater_robot)

// チームカラーに対する特殊化がちゃんと機能しているか
BOOST_AUTO_TEST_CASE(color, *boost::unit_test::tolerance(0.0000001)) {
  model::updater::robot<model::team_color::blue> rbu;
  model::updater::robot<model::team_color::yellow> ryu;

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);

    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(1);
    rb1->set_x(10);
    rb1->set_y(20);
    rb1->set_orientation(rad(30));
    rb1->set_confidence(90.0);

    auto ry2 = f.add_robots_yellow();
    ry2->set_robot_id(2);
    ry2->set_x(40);
    ry2->set_y(50);
    ry2->set_orientation(rad(60));
    ry2->set_confidence(90.0);

    rbu.update(f);
    ryu.update(f);
  }

  {
    ai_server::model::robot r;

    // 青ロボット用updaterから青ロボットの値が取れる
    const auto rb = rbu.value();
    BOOST_TEST(rb.size() == 1);
    BOOST_CHECK_NO_THROW(r = rb.at(1));
    BOOST_TEST(r.x() == 10);
    BOOST_TEST(r.y() == 20);
    BOOST_TEST(r.theta() == rad(30));

    // 黄ロボット用updaterから黄ロボットの値が取れる
    const auto ry = ryu.value();
    BOOST_TEST(ry.size() == 1);
    BOOST_CHECK_NO_THROW(r = ry.at(2));
    BOOST_TEST(r.x() == 40);
    BOOST_TEST(r.y() == 50);
    BOOST_TEST(r.theta() == rad(60));
  }
}

BOOST_AUTO_TEST_CASE(normal, *boost::unit_test::tolerance(0.0000001)) {
  model::updater::robot<model::team_color::blue> ru;

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);

    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(1);
    rb1->set_x(10);
    rb1->set_y(11);
    rb1->set_orientation(rad(12));
    rb1->set_confidence(94.0);

    auto rb3 = f.add_robots_blue();
    rb3->set_robot_id(3);
    rb3->set_x(30);
    rb3->set_y(31);
    rb3->set_orientation(rad(32));
    rb3->set_confidence(95.0);

    auto rb5 = f.add_robots_blue();
    rb5->set_robot_id(5);
    rb5->set_x(50);
    rb5->set_y(51);
    rb5->set_orientation(rad(52));
    rb5->set_confidence(96.0);

    ru.update(f);
  }

  {
    ai_server::model::robot r;
    const auto rb = ru.value();

    // 検出された青ロボは3台
    BOOST_TEST(rb.size() == 3);

    // ID1の青ロボが存在 (ID1の要素を参照してもstd::out_of_range例外が飛ばない)
    BOOST_CHECK_NO_THROW(r = rb.at(1));
    BOOST_TEST(r.x() == 10);
    BOOST_TEST(r.y() == 11);
    BOOST_TEST(r.theta() == rad(12));

    // ID3の青ロボが存在
    BOOST_CHECK_NO_THROW(r = rb.at(3));
    BOOST_TEST(r.x() == 30);
    BOOST_TEST(r.y() == 31);
    BOOST_TEST(r.theta() == rad(32));

    // ID5の青ロボが存在
    BOOST_CHECK_NO_THROW(r = rb.at(5));
    BOOST_TEST(r.x() == 50);
    BOOST_TEST(r.y() == 51);
    BOOST_TEST(r.theta() == rad(52));
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(1);

    // 最初よりもconfidenceが高いデータ
    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(1);
    rb1->set_x(13);
    rb1->set_y(14);
    rb1->set_orientation(rad(15));
    rb1->set_confidence(95.0);

    // 新たに検出されたIDのデータ
    auto rb2 = f.add_robots_blue();
    rb2->set_robot_id(2);
    rb2->set_x(20);
    rb2->set_y(21);
    rb2->set_orientation(rad(22));
    rb2->set_confidence(94.0);

    // 最初よりもconfidenceが低いデータ
    auto rb5 = f.add_robots_blue();
    rb5->set_robot_id(5);
    rb5->set_x(53);
    rb5->set_y(54);
    rb5->set_orientation(rad(55));
    rb5->set_confidence(93.0);

    ru.update(f);
  }

  {
    ai_server::model::robot r;
    const auto rb = ru.value();

    // 検出された青ロボは4台
    BOOST_TEST(rb.size() == 4);

    // ID1の青ロボが存在
    // cam1で検出されたID1の青ロボのほうがconfidenceが高いので値が更新される
    BOOST_CHECK_NO_THROW(r = rb.at(1));
    BOOST_TEST(r.x() == 13);
    BOOST_TEST(r.y() == 14);
    BOOST_TEST(r.theta() == rad(15));

    // ID2の青ロボが存在
    BOOST_CHECK_NO_THROW(r = rb.at(2));
    BOOST_TEST(r.x() == 20);
    BOOST_TEST(r.y() == 21);
    BOOST_TEST(r.theta() == rad(22));

    // ID3の青ロボが存在, 値の変化なし
    BOOST_CHECK_NO_THROW(r = rb.at(3));
    BOOST_TEST(r.x() == 30);
    BOOST_TEST(r.y() == 31);
    BOOST_TEST(r.theta() == rad(32));

    // ID5の青ロボが存在
    // cam0で検出されたID5の青ロボのほうがconfidenceが高いので値の変化なし
    BOOST_CHECK_NO_THROW(r = rb.at(5));
    BOOST_TEST(r.x() == 50);
    BOOST_TEST(r.y() == 51);
    BOOST_TEST(r.theta() == rad(52));
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);

    ru.update(f);
  }

  {
    ai_server::model::robot r;
    const auto rb = ru.value();

    // 検出された青ロボは3台
    BOOST_TEST(rb.size() == 3);

    // cam1で検出されたID1の青ロボが存在
    BOOST_CHECK_NO_THROW(r = rb.at(1));
    BOOST_TEST(r.x() == 13);
    BOOST_TEST(r.y() == 14);
    BOOST_TEST(r.theta() == rad(15));

    // cam1で検出されたID2の青ロボが存在
    BOOST_CHECK_NO_THROW(r = rb.at(2));
    BOOST_TEST(r.x() == 20);
    BOOST_TEST(r.y() == 21);
    BOOST_TEST(r.theta() == rad(22));

    // ID5の青ロボが存在
    // cam0から検出されなくなったが, まだcam1に残っているので
    BOOST_CHECK_NO_THROW(r = rb.at(5));
    BOOST_TEST(r.x() == 50);
    BOOST_TEST(r.y() == 51);
    BOOST_TEST(r.theta() == rad(52));
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(1);

    ru.update(f);
  }

  {
    const auto rb = ru.value();

    // そして誰もいなくなった
    BOOST_TEST(rb.size() == 0);
  }
}

BOOST_AUTO_TEST_CASE(transformation, *boost::unit_test::tolerance(0.0000001)) {
  using namespace boost::math::double_constants;

  model::updater::robot<model::team_color::blue> ru;
  // 90度回転, x軸方向に10, y軸方向に20平行移動
  ru.set_transformation_matrix(util::math::make_transformation_matrix(10.0, 20.0, half_pi));

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(0);
    rb1->set_x(100);
    rb1->set_y(200);
    rb1->set_orientation(0);
    rb1->set_confidence(90.0);

    ru.update(f);
  }

  {
    const auto rb = ru.value();
    ai_server::model::robot r;
    BOOST_TEST(rb.size() == 1);
    BOOST_CHECK_NO_THROW(r = rb.at(0));
    BOOST_TEST(r.x() == -190.0);
    BOOST_TEST(r.y() == 120.0);
    BOOST_TEST(r.theta() == 3 * half_pi);
  }
}

struct mock_filter1 : public filter::base<model::robot, filter::timing::same> {
  // コンストラクタの引数に渡された値
  int arg1;
  int arg2;

  // 最後にupdateの引数に与えられた値
  std::optional<model::robot> v;
  std::chrono::system_clock::time_point t;

  mock_filter1(int a1, int a2) : arg1(a1), arg2(a2) {}

  std::optional<model::robot> update(std::optional<model::robot> value,
                                     std::chrono::system_clock::time_point time) override {
    v = value;
    t = time;

    if (v.has_value()) {
      // vx, ayに適当な値をセットして返す
      model::robot r{};
      r.set_vx(value->x() * 2);
      r.set_ay(value->y() * 3);
      return r;
    } else {
      return std::nullopt;
    }
  }
};

BOOST_AUTO_TEST_CASE(filter_same, *boost::unit_test::tolerance(0.0000001)) {
  model::updater::robot<model::team_color::blue> ru;

  // ID0にmock_filter1を設定
  const auto fp = ru.set_filter<mock_filter1>(0, 123, 456).lock();

  // set_filterの引数に与えた値がFilterのコンストラクタに正しく渡されているか
  BOOST_TEST(fp->arg1 == 123);
  BOOST_TEST(fp->arg2 == 456);

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(0);
    rb1->set_x(1);
    rb1->set_y(2);
    rb1->set_orientation(rad(3));
    rb1->set_confidence(90.0);

    ru.update(f);
  }

  {
    // updateの引数に値が正しく渡されているか
    BOOST_TEST(fp->v->x() == 1);
    BOOST_TEST(fp->v->y() == 2);
    BOOST_TEST(fp->v->theta() == rad(3));
    BOOST_TEST(fp->t.time_since_epoch().count() == dc(2s).count());

    // vx, ayが選択された値の2倍, 3倍になっている
    const auto rb = ru.value();
    ai_server::model::robot r;
    BOOST_TEST(rb.size() == 1);
    BOOST_CHECK_NO_THROW(r = rb.at(0));
    BOOST_TEST(r.vx() == 2);
    BOOST_TEST(r.ay() == 6);
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(1);
    f.set_t_capture(4.0);

    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(0);
    rb1->set_x(10);
    rb1->set_y(20);
    rb1->set_orientation(rad(30));
    rb1->set_confidence(92.0);

    ru.update(f);
  }

  {
    // updateの引数に値が正しく渡されているか
    BOOST_TEST(fp->v->x() == 10);
    BOOST_TEST(fp->v->y() == 20);
    BOOST_TEST(fp->v->theta() == rad(30));
    BOOST_TEST(fp->t.time_since_epoch().count() == dc(4s).count());

    // vx, ayが選択された値の2倍, 3倍になっている
    const auto rb = ru.value();
    ai_server::model::robot r;
    BOOST_TEST(rb.size() == 1);
    BOOST_CHECK_NO_THROW(r = rb.at(0));
    BOOST_TEST(r.vx() == 20);
    BOOST_TEST(r.ay() == 60);
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(8.0);

    auto rb1 = f.add_robots_blue();
    rb1->set_x(100);
    rb1->set_y(200);
    rb1->set_orientation(rad(300));
    rb1->set_confidence(90.0);

    ru.update(f);
  }

  {
    // 最新でない値が選択された場合は変化しない
    const auto rb = ru.value();
    ai_server::model::robot r;
    BOOST_TEST(rb.size() == 1);
    BOOST_CHECK_NO_THROW(r = rb.at(0));
    BOOST_TEST(r.vx() == 20);
    BOOST_TEST(r.ay() == 60);
  }

  {
    ssl_protos::vision::Frame f1;
    f1.set_camera_id(1);
    f1.set_t_capture(10.0);

    ru.update(f1);

    ssl_protos::vision::Frame f2;
    f2.set_camera_id(0);
    f2.set_t_capture(10.0);

    ru.update(f2);
  }

  {
    // 対象がロストしたらnulloptが渡されている
    BOOST_TEST(!fp->v.has_value());
    BOOST_TEST(fp->t.time_since_epoch().count() == dc(10s).count());

    // Filter が nullopt を返したら対象のデータは削除される
    const auto rb = ru.value();
    BOOST_TEST(rb.size() == 0);
  }
}

struct mock_filter2 : public filter::base<model::robot, filter::timing::same> {
  std::optional<model::robot> update(std::optional<model::robot> value,
                                     std::chrono::system_clock::time_point) override {
    // 引数に値が渡された時にnullopt, そうでない時に値を返すFilter
    if (value.has_value()) {
      return std::nullopt;
    } else {
      return model::robot{123, 456, 2};
    }
  }
};

BOOST_AUTO_TEST_CASE(filter_same2) {
  model::updater::robot<model::team_color::blue> ru;

  {
    // 初期状態
    const auto rb = ru.value();
    BOOST_TEST(rb.size() == 0);
  }

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(0);
    rb1->set_x(1);
    rb1->set_y(2);
    rb1->set_orientation(rad(3));
    rb1->set_confidence(90.0);

    ru.update(f);

    const auto rb = ru.value();
    BOOST_TEST(rb.size() == 1);
    BOOST_TEST(rb.count(0) == 1);
  }

  // ID0, ID1にmock_filter2を設定
  const auto fp1 = ru.set_filter<mock_filter2>(0).lock();
  const auto fp2 = ru.set_filter<mock_filter2>(1).lock();

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(4.0);

    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(1);
    rb1->set_x(1);
    rb1->set_y(2);
    rb1->set_orientation(rad(3));
    rb1->set_confidence(90.0);

    ru.update(f);

    const auto rb = ru.value();
    BOOST_TEST(rb.size() == 1);
    ai_server::model::robot r;
    BOOST_CHECK_NO_THROW(r = rb.at(0));
    BOOST_TEST(r.x() == 123);
    BOOST_TEST(r.y() == 456);
    BOOST_TEST(r.theta() == 2);
  }
}

struct mock_filter3 : public filter::base<model::robot, filter::timing::manual> {
  // コンストラクタの引数に渡された値
  int arg1;
  int arg2;

  // set_raw_value で渡された値
  std::optional<model::robot> value;
  std::chrono::system_clock::time_point time;

  mock_filter3(std::recursive_mutex& mutex, mock_filter3::writer_func_type wf, int a1, int a2)
      : base(mutex, wf), arg1(a1), arg2(a2) {}

  void set_raw_value(std::optional<model::robot> v,
                     std::chrono::system_clock::time_point t) override {
    std::unique_lock lock{mutex()};
    value = v;
    time  = t;
  }

  void wv(std::optional<model::robot> v) {
    std::unique_lock lock{mutex()};
    write(v);
  }
};

BOOST_AUTO_TEST_CASE(manual_filter, *boost::unit_test::tolerance(0.0000001)) {
  model::updater::robot<model::team_color::blue> ru;

  // ID0にmock_filter3を設定
  const auto fp = ru.set_filter<mock_filter3>(0, 123, 456).lock();

  // set_filterの引数に与えた値がFilterのコンストラクタに正しく渡されているか
  BOOST_TEST(fp->arg1 == 123);
  BOOST_TEST(fp->arg2 == 456);

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(0);
    rb1->set_x(10);
    rb1->set_y(20);
    rb1->set_orientation(rad(30));
    rb1->set_confidence(90.0);

    ru.update(f);
  }

  {
    // updateが呼ばれただけでは値は更新されない
    const auto rb = ru.value();
    BOOST_TEST(rb.size() == 0);

    // set_raw_value から観測値が渡されている
    const auto lv = fp->value;
    BOOST_TEST(lv->x() == 10);
    BOOST_TEST(lv->y() == 20);
    BOOST_TEST(lv->theta() == rad(30));

    const auto lt = fp->time;
    BOOST_TEST(lt.time_since_epoch().count() == dc(2s).count());
  }

  {
    // writeで値が更新される
    const auto r1 = model::robot{40, 50, 60};
    fp->wv(r1);

    const auto rb = ru.value();
    ai_server::model::robot r2;
    BOOST_TEST(rb.size() == 1);
    BOOST_CHECK_NO_THROW(r2 = rb.at(0));
    BOOST_TEST(r2.x() == r1.x());
    BOOST_TEST(r2.y() == r1.y());
    BOOST_TEST(r2.theta() == r1.theta());
  }

  {
    // 候補リストを空にするとset_raw_valueにnulloptが渡されている
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(4.0);
    ru.update(f);

    const auto lv = fp->value;
    BOOST_TEST(!lv.has_value());
    const auto lt = fp->time;
    BOOST_TEST(lt.time_since_epoch().count() == dc(4s).count());
  }

  {
    // nulloptで更新すると消える
    fp->wv(std::nullopt);

    const auto rb = ru.value();
    BOOST_TEST(rb.size() == 0);
  }
}

BOOST_AUTO_TEST_CASE(default_filter, *boost::unit_test::tolerance(0.0000001)) {
  model::updater::robot<model::team_color::blue> ru;

  // ID1にmock_filter3を設定
  const auto fp1 = ru.set_filter<mock_filter3>(1, 123, 456);
  // デフォルトのFilterとしてmock_filter1を設定
  ru.set_default_filter<mock_filter1>(123, 456);

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);

    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(1);
    rb1->set_x(10);
    rb1->set_y(11);
    rb1->set_orientation(rad(12));
    rb1->set_confidence(94.0);

    auto rb3 = f.add_robots_blue();
    rb3->set_robot_id(3);
    rb3->set_x(30);
    rb3->set_y(31);
    rb3->set_orientation(rad(32));
    rb3->set_confidence(95.0);

    auto rb5 = f.add_robots_blue();
    rb5->set_robot_id(5);
    rb5->set_x(50);
    rb5->set_y(51);
    rb5->set_orientation(rad(52));
    rb5->set_confidence(96.0);

    ru.update(f);
  }

  {
    ai_server::model::robot r;
    const auto rb = ru.value();

    // 検出された青ロボは2台
    BOOST_TEST(rb.size() == 2);

    // ID1の青ロボが存在しない
    // (検出はされているが, manualなFilterが登録されているので)
    BOOST_TEST(!rb.count(1));

    // ID3の青ロボが存在
    // mock_filter1が適用された値になっている
    BOOST_CHECK_NO_THROW(r = rb.at(3));
    BOOST_TEST(r.vx() == 60);
    BOOST_TEST(r.ay() == 93);

    // ID5の青ロボが存在
    // mock_filter1が適用された値になっている
    BOOST_CHECK_NO_THROW(r = rb.at(5));
    BOOST_TEST(r.vx() == 100);
    BOOST_TEST(r.ay() == 153);
  }

  // default_filterを解除
  ru.clear_default_filter();

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(1);

    auto rb7 = f.add_robots_blue();
    rb7->set_robot_id(7);
    rb7->set_x(70);
    rb7->set_y(71);
    rb7->set_orientation(rad(72));
    rb7->set_confidence(94.0);

    ru.update(f);
  }

  {
    ai_server::model::robot r;
    const auto rb = ru.value();

    // 検出された青ロボは3台
    BOOST_TEST(rb.size() == 3);

    // ID1の青ロボが存在しない
    BOOST_TEST(!rb.count(1));

    // ID3の青ロボが存在
    BOOST_CHECK_NO_THROW(r = rb.at(3));
    BOOST_TEST(r.vx() == 60);
    BOOST_TEST(r.ay() == 93);

    // ID5の青ロボが存在
    BOOST_CHECK_NO_THROW(r = rb.at(5));
    BOOST_TEST(r.vx() == 100);
    BOOST_TEST(r.ay() == 153);

    // ID7の青ロボが存在
    // filter_initializer_が初期化されたので, mock_filter1は適用されていない
    BOOST_CHECK_NO_THROW(r = rb.at(7));
    BOOST_TEST(r.x() == 70);
    BOOST_TEST(r.y() == 71);
    BOOST_TEST(r.theta() == rad(72));
  }
}

BOOST_AUTO_TEST_CASE(clear_filter) {
  model::updater::robot<model::team_color::blue> ru;

  // clear_filterを呼ぶと登録したFilterが死んでいる
  const auto fp1 = ru.set_filter<mock_filter1>(0, 123, 456);
  BOOST_TEST(!fp1.expired());
  ru.clear_filter(0);
  BOOST_TEST(fp1.expired());

  // manualなFilterも同様
  const auto fp2 = ru.set_filter<mock_filter3>(0, 123, 456);
  BOOST_TEST(!fp2.expired());
  ru.clear_filter(0);
  BOOST_TEST(fp2.expired());

  // sameなFilterが登録されている状態でmanualなFilterを登録するとsameなFilterは死ぬ
  const auto fp3 = ru.set_filter<mock_filter1>(0, 123, 456);
  const auto fp4 = ru.set_filter<mock_filter3>(0, 123, 456);
  BOOST_TEST(fp3.expired());
  BOOST_TEST(!fp4.expired());

  // 逆も同様
  const auto fp5 = ru.set_filter<mock_filter1>(0, 123, 456);
  BOOST_TEST(fp4.expired());
  BOOST_TEST(!fp5.expired());
  ru.clear_filter(0);

  // いくつかのFilterを設定
  const auto fp6 = ru.set_filter<mock_filter1>(0, 123, 456);
  const auto fp7 = ru.set_filter<mock_filter1>(1, 123, 456);
  const auto fp8 = ru.set_filter<mock_filter3>(2, 123, 456);
  const auto fp9 = ru.set_filter<mock_filter3>(3, 123, 456);
  BOOST_TEST(!fp6.expired());
  BOOST_TEST(!fp7.expired());
  BOOST_TEST(!fp8.expired());
  BOOST_TEST(!fp9.expired());
  // clear_all_filters()で全部死ぬ
  ru.clear_all_filters();
  BOOST_TEST(fp6.expired());
  BOOST_TEST(fp7.expired());
  BOOST_TEST(fp8.expired());
  BOOST_TEST(fp9.expired());
}

// set_raw_value() で write() を呼ぶ filter
struct mock_filter4 : public filter::base<model::robot, filter::timing::manual> {
  mock_filter4(std::recursive_mutex& mutex, mock_filter4::writer_func_type wf)
      : base(mutex, wf) {}

  void set_raw_value(std::optional<model::robot> v,
                     std::chrono::system_clock::time_point) override {
    std::unique_lock lock{mutex()};
    write(v);
  }

  void wv(std::optional<model::robot> v) {
    std::unique_lock lock{mutex()};
    write(v);
  }
};

BOOST_AUTO_TEST_CASE(manual_filter2, *boost::unit_test::tolerance(0.0000001)) {
  model::updater::robot<model::team_color::blue> ru;
  const auto fp = ru.set_filter<mock_filter4>(0).lock();

  {
    ssl_protos::vision::Frame f;
    f.set_camera_id(0);
    f.set_t_capture(2.0);

    auto rb1 = f.add_robots_blue();
    rb1->set_robot_id(0);
    rb1->set_x(10);
    rb1->set_y(20);
    rb1->set_orientation(rad(30));
    rb1->set_confidence(90.0);

    ru.update(f);
  }

  {
    // ID0 が見える
    const auto rb = ru.value();
    BOOST_TEST(rb.size() == 1);

    ai_server::model::robot r;
    BOOST_CHECK_NO_THROW(r = rb.at(0));
    BOOST_TEST(r.x() == 10);
    BOOST_TEST(r.y() == 20);
    BOOST_TEST(r.theta() == rad(30));
  }

  {
    // writeで値が更新される
    fp->wv(model::robot{40, 50, rad(60)});

    // ID0 が見える
    const auto rb = ru.value();
    BOOST_TEST(rb.size() == 1);

    ai_server::model::robot r;
    BOOST_CHECK_NO_THROW(r = rb.at(0));
    BOOST_TEST(r.x() == 40);
    BOOST_TEST(r.y() == 50);
    BOOST_TEST(r.theta() == rad(60));
  }
}

BOOST_AUTO_TEST_SUITE_END()
