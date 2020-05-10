#define BOOST_TEST_DYN_LINK

#include <stdexcept>

#include <boost/test/unit_test.hpp>

#include "ai_server/game/nnabla.h"

BOOST_AUTO_TEST_SUITE(nnabla)

BOOST_AUTO_TEST_CASE(nnp1) {
  ai_server::game::nnabla n{{"cpu"},
                            "0",
                            {
                                {"hoge", {"hoge.nnp", false}},
                            }};

  {
    auto [ctx, path] = n.nnp("hoge", "float", true);

    // backend に与えたデータ型が付いている
    BOOST_TEST(ctx.backend == (std::vector{"cpu:float"}), boost::test_tools::per_element());
    // CpuCachedArray が返ってくる
    BOOST_TEST(ctx.array_class == "CpuCachedArray");
    BOOST_TEST(ctx.device_id == "0");

    BOOST_TEST(path == "hoge.nnp");
  }

  {
    auto [ctx, path] = n.nnp("hoge", "nyan", false);

    // backend に与えたデータ型が付いている
    BOOST_TEST(ctx.backend == (std::vector{"cpu:nyan"}), boost::test_tools::per_element());
    // Cached でない array_class になっている
    BOOST_TEST(ctx.array_class == "CpuArray");
    BOOST_TEST(ctx.device_id == "0");

    BOOST_TEST(path == "hoge.nnp");
  }

  // 現在の仕様では存在しない key を渡したときに例外が飛ぶ
  BOOST_CHECK_THROW(n.nnp("sonzai-shinai", "float", true), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(nnp2) {
  ai_server::game::nnabla n{{"cudnn", "cuda", "cpu"},
                            "1",
                            {
                                {"hoge", {"hoge.nnp", false}},
                                {"fuga", {"fuga.nnp", true}},
                            }};

  {
    auto [ctx, path] = n.nnp("hoge", "float", true);

    BOOST_TEST(ctx.backend == (std::vector{"cudnn:float", "cuda:float", "cpu:float"}),
               boost::test_tools::per_element());
    BOOST_TEST(ctx.array_class == "CudaCachedArray");
    BOOST_TEST(ctx.device_id == "1");

    BOOST_TEST(path == "hoge.nnp");
  }

  {
    auto [ctx, path] = n.nnp("hoge", "float", false);

    BOOST_TEST(ctx.backend == (std::vector{"cudnn:float", "cuda:float", "cpu:float"}),
               boost::test_tools::per_element());
    BOOST_TEST(ctx.array_class == "CudaArray");
    BOOST_TEST(ctx.device_id == "1");

    BOOST_TEST(path == "hoge.nnp");
  }

  {
    auto [ctx, path] = n.nnp("fuga", "float", true);

    // fuga は常に CPU を使うフラグが true なので "cpu:<data_type>" のみ
    BOOST_TEST(ctx.backend == (std::vector{"cpu:float"}), boost::test_tools::per_element());
    // array_class も CPU のものになっている
    BOOST_TEST(ctx.array_class == "CpuCachedArray");
    BOOST_TEST(ctx.device_id == "1");

    BOOST_TEST(path == "fuga.nnp");
  }
  {
    auto [ctx, path] = n.nnp("fuga", "float", false);

    BOOST_TEST(ctx.backend == (std::vector{"cpu:float"}), boost::test_tools::per_element());
    BOOST_TEST(ctx.array_class == "CpuArray");
    BOOST_TEST(ctx.device_id == "1");

    BOOST_TEST(path == "fuga.nnp");
  }
}

BOOST_AUTO_TEST_SUITE_END()
