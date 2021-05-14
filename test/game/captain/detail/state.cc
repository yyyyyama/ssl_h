#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>

#include "ai_server/game/captain/detail/state.h"

namespace cd    = ai_server::game::captain::detail;
namespace model = ai_server::model;

using team_color = model::team_color;
using rcmd       = model::refbox::game_command;
using refstage   = model::refbox::stage_name;

BOOST_AUTO_TEST_SUITE(state)

// 基本的 (単純) な状態遷移
BOOST_AUTO_TEST_CASE(basic) {
  cd::state s{model::team_color::yellow};

  // 初期状態は unknown
  BOOST_TEST((s.current_situation() == cd::situation::unknown));

  BOOST_TEST((s.update({}, rcmd::halt) == cd::situation::halt));
  BOOST_TEST((s.current_situation() == cd::situation::halt));

  BOOST_TEST((s.update({}, rcmd::stop) == cd::situation::stop));
  BOOST_TEST((s.current_situation() == cd::situation::stop));

  BOOST_TEST((s.update({}, rcmd::force_start) == cd::situation::force_start));
  BOOST_TEST((s.current_situation() == cd::situation::force_start));

  BOOST_TEST((s.update({}, rcmd::timeout_yellow) == cd::situation::timeout));
  BOOST_TEST((s.current_situation() == cd::situation::timeout));

  BOOST_TEST((s.update({}, rcmd::timeout_blue) == cd::situation::timeout));
  BOOST_TEST((s.current_situation() == cd::situation::timeout));

  // 関係のない状態からの normal_start は直前の situation を継続する
  BOOST_TEST((s.update({}, rcmd::normal_start) == cd::situation::timeout));

  // 知らないコマンドが来たら unknown
  BOOST_TEST((s.update({}, static_cast<rcmd>(0x12345678)) == cd::situation::unknown));
}

// チーム特有の状態遷移
#define AI_SERVER_TEST_CASE_FOR_STATE(ally, enemy)                                             \
  BOOST_AUTO_TEST_CASE(ally) {                                                                 \
    cd::state s{model::team_color::ally};                                                      \
                                                                                               \
    /* prepare_kickoff -> normal_start */                                                      \
    BOOST_TEST((s.update({}, rcmd::prepare_kickoff_##ally) == cd::situation::kickoff_attack)); \
    BOOST_TEST((s.current_situation() == cd::situation::kickoff_attack));                      \
    BOOST_TEST((s.update({}, rcmd::normal_start) == cd::situation::kickoff_attack_start));     \
    BOOST_TEST((s.current_situation() == cd::situation::kickoff_attack_start));                \
    BOOST_TEST((s.update({}, rcmd::normal_start) == cd::situation::kickoff_attack_start));     \
    BOOST_TEST((s.current_situation() == cd::situation::kickoff_attack_start));                \
                                                                                               \
    BOOST_TEST(                                                                                \
        (s.update({}, rcmd::prepare_kickoff_##enemy) == cd::situation::kickoff_defense));      \
    BOOST_TEST((s.current_situation() == cd::situation::kickoff_defense));                     \
    BOOST_TEST((s.update({}, rcmd::normal_start) == cd::situation::kickoff_defense));          \
    BOOST_TEST((s.current_situation() == cd::situation::kickoff_defense));                     \
    BOOST_TEST((s.update({}, rcmd::normal_start) == cd::situation::kickoff_defense));          \
    BOOST_TEST((s.current_situation() == cd::situation::kickoff_defense));                     \
                                                                                               \
    constexpr auto fh = refstage::normal_first_half;                                           \
    constexpr auto so = refstage::penalty_shootout;                                            \
                                                                                               \
    /* prepare_penalty -> normal_start */                                                      \
    BOOST_TEST((s.update(fh, rcmd::prepare_penalty_##ally) == cd::situation::penalty_attack)); \
    BOOST_TEST((s.current_situation() == cd::situation::penalty_attack));                      \
    BOOST_TEST((s.update(fh, rcmd::normal_start) == cd::situation::penalty_attack_start));     \
    BOOST_TEST((s.current_situation() == cd::situation::penalty_attack_start));                \
    BOOST_TEST((s.update(fh, rcmd::normal_start) == cd::situation::penalty_attack_start));     \
    BOOST_TEST((s.current_situation() == cd::situation::penalty_attack_start));                \
                                                                                               \
    BOOST_TEST(                                                                                \
        (s.update(fh, rcmd::prepare_penalty_##enemy) == cd::situation::penalty_defense));      \
    BOOST_TEST((s.current_situation() == cd::situation::penalty_defense));                     \
    BOOST_TEST((s.update(fh, rcmd::normal_start) == cd::situation::penalty_defense));          \
    BOOST_TEST((s.current_situation() == cd::situation::penalty_defense));                     \
    BOOST_TEST((s.update(fh, rcmd::normal_start) == cd::situation::penalty_defense));          \
    BOOST_TEST((s.current_situation() == cd::situation::penalty_defense));                     \
                                                                                               \
    BOOST_TEST(                                                                                \
        (s.update(so, rcmd::prepare_penalty_##ally) == cd::situation::shootout_attack));       \
    BOOST_TEST((s.current_situation() == cd::situation::shootout_attack));                     \
    BOOST_TEST((s.update(so, rcmd::normal_start) == cd::situation::shootout_attack_start));    \
    BOOST_TEST((s.current_situation() == cd::situation::shootout_attack_start));               \
    BOOST_TEST((s.update(so, rcmd::normal_start) == cd::situation::shootout_attack_start));    \
    BOOST_TEST((s.current_situation() == cd::situation::shootout_attack_start));               \
                                                                                               \
    BOOST_TEST(                                                                                \
        (s.update(so, rcmd::prepare_penalty_##enemy) == cd::situation::shootout_defense));     \
    BOOST_TEST((s.current_situation() == cd::situation::shootout_defense));                    \
    BOOST_TEST((s.update(so, rcmd::normal_start) == cd::situation::shootout_defense_start));   \
    BOOST_TEST((s.current_situation() == cd::situation::shootout_defense_start));              \
    BOOST_TEST((s.update(so, rcmd::normal_start) == cd::situation::shootout_defense_start));   \
    BOOST_TEST((s.current_situation() == cd::situation::shootout_defense_start));              \
                                                                                               \
    /* direct/indirect */                                                                      \
    BOOST_TEST((s.update({}, rcmd::direct_free_##ally) == cd::situation::setplay_attack));     \
    BOOST_TEST((s.current_situation() == cd::situation::setplay_attack));                      \
                                                                                               \
    BOOST_TEST((s.update({}, rcmd::direct_free_##enemy) == cd::situation::setplay_defense));   \
    BOOST_TEST((s.current_situation() == cd::situation::setplay_defense));                     \
                                                                                               \
    BOOST_TEST((s.update({}, rcmd::indirect_free_##ally) == cd::situation::setplay_attack));   \
    BOOST_TEST((s.current_situation() == cd::situation::setplay_attack));                      \
                                                                                               \
    BOOST_TEST((s.update({}, rcmd::indirect_free_##enemy) == cd::situation::setplay_defense)); \
    BOOST_TEST((s.current_situation() == cd::situation::setplay_defense));                     \
                                                                                               \
    /* ball_placement */                                                                       \
    BOOST_TEST((s.update({}, rcmd::ball_placement_##ally) == cd::situation::ball_placement));  \
    BOOST_TEST((s.current_situation() == cd::situation::ball_placement));                      \
                                                                                               \
    BOOST_TEST(                                                                                \
        (s.update({}, rcmd::ball_placement_##enemy) == cd::situation::ball_placement_enemy));  \
    BOOST_TEST((s.current_situation() == cd::situation::ball_placement_enemy));                \
  }

AI_SERVER_TEST_CASE_FOR_STATE(yellow, blue)
AI_SERVER_TEST_CASE_FOR_STATE(blue, yellow)

BOOST_AUTO_TEST_SUITE_END()
