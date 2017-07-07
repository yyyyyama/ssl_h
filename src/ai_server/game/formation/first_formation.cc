#include "first_formation.h"

namespace ai_server {
namespace game {
namespace formation {

first_formation::first_formation(const model::world& world, const model::refbox& refcommand,
                                 bool is_yellow, const std::vector<unsigned int>& ids)
    : base(world, is_yellow, refcommand),
      ids_(ids),
      kicked_flag_(false),
      regular_flag_(true),
      previous_ball_(world.ball()) {}

std::vector<std::shared_ptr<agent::base>> first_formation::execute() {
  const auto command = refcommand_.command();
  current_command_   = convert_command(command);

  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();

  exe_.clear();

  //壁を決定する,変更があった場合にはothers_も更新する
  update_keeper();
  decide_wall();
  //壁,キッカーなどの役割を決定
  if (current_command_ == command::stop && is_command_changed()) {
    decide_kicker();
    except_keeper();
    waiter();
  }

  //全て見えない場合は何もしない
  if (std::none_of(ids_.begin(), ids_.end(),
                   [&our_robots](auto&& id) { return our_robots.count(id); })) {
    std::vector<std::shared_ptr<agent::base>> dummy;
    return dummy;
  }

  // commandによってagentを使い分ける
  switch (current_command_) {
    case command::halt:
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        halt_ = halt();
      }
      exe_.push_back(halt_);
      break;

    case command::stop:
      if (is_command_changed()) {
        stop_         = stop();
        df_           = defense(agent::defense::defense_mode::normal_mode, false);
        kicked_flag_  = false;
        regular_flag_ = true;
      }
      exe_.push_back(stop_);
      exe_.push_back(df_);
      break;

    case command::kickoff_attack_start:
      //定常状態に入る
      if (kickoff_->finished()) {
        if (regular_flag_) { //定常状態に入った時に初期化、regular_flag_をfalseにしておく
          regular_      = regular(true);
          df_           = defense(agent::defense::defense_mode::normal_mode, false);
          regular_flag_ = false;
        }
        exe_.push_back(regular_);
        exe_.push_back(df_);
        break;
      }

      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        kickoff_        = kickoff(true);
        kickoff_waiter_ = kickoff_waiter(agent::kick_off_waiter::kickoff_mode::attack, true);
        df_             = defense(agent::defense::defense_mode::normal_mode, false);
        regular_flag_   = true;
      }
      exe_.push_back(kickoff_);
      exe_.push_back(kickoff_waiter_);
      exe_.push_back(df_);
      break;

    case command::penalty_attack_start:
      //定常状態に入る
      if (pk_->finished()) {
        if (regular_flag_) {
          regular_      = regular(true);
          df_           = defense(agent::defense::defense_mode::normal_mode, false);
          regular_flag_ = false;
        }
        exe_.push_back(regular_);
        exe_.push_back(df_);
        break;
      }

      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        pk_           = pk(true, true);
        df_           = defense(agent::defense::defense_mode::normal_mode, false);
        regular_flag_ = true;
      }
      exe_.push_back(pk_);
      exe_.push_back(df_);
      break;

    case command::force_start:
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_ = regular(true);
        df_      = defense(agent::defense::defense_mode::normal_mode, false);
      }
      exe_.push_back(regular_);
      exe_.push_back(df_);
      break;

    case command::kickoff_attack:
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        kickoff_        = kickoff(false);
        kickoff_waiter_ = kickoff_waiter(agent::kick_off_waiter::kickoff_mode::attack, true);
        df_             = defense(agent::defense::defense_mode::normal_mode, false);
      }
      exe_.push_back(kickoff_);
      exe_.push_back(kickoff_waiter_);
      exe_.push_back(df_);
      break;

    case command::kickoff_defense:
      //定常状態に入る
      if (kicked_flag_) {
        if (regular_flag_) {
          regular_      = regular(true);
          df_           = defense(agent::defense::defense_mode::normal_mode, false);
          regular_flag_ = false;
        }
        exe_.push_back(regular_);
        exe_.push_back(df_);
        break;
      }

      kicked_flag_ = kicked(ball);
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        kickoff_waiter_ = kickoff_waiter(agent::kick_off_waiter::kickoff_mode::defense, false);
        df_             = defense(agent::defense::defense_mode::normal_mode, false);
        regular_flag_   = true;
        kicked_flag_    = false;
      }
      exe_.push_back(kickoff_waiter_);
      exe_.push_back(df_);
      break;

    case command::penalty_attack:
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        pk_ = pk(false, true);
        df_ = defense(agent::defense::defense_mode::normal_mode, false);
      }
      exe_.push_back(pk_);
      exe_.push_back(df_);
      break;

    case command::penalty_defense:
      //定常状態に入る
      if (kicked_flag_) {
        if (regular_flag_) {
          regular_      = regular(true);
          df_           = defense(agent::defense::defense_mode::normal_mode, false);
          regular_flag_ = false;
        }
        exe_.push_back(regular_);
        exe_.push_back(df_);
        break;
      }

      kicked_flag_ = kicked(ball);
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        pk_           = pk(false, false);
        df_           = defense(agent::defense::defense_mode::pk_mode, false);
        regular_flag_ = true;
        kicked_flag_  = false;
      }
      exe_.push_back(pk_);
      exe_.push_back(df_);
      break;

    case command::setplay_attack:
      ////定常状態に入る
      if (kicked_flag_) {
        if (regular_flag_) {
          regular_      = regular(true);
          df_           = defense(agent::defense::defense_mode::normal_mode, false);
          regular_flag_ = false;
        }
        exe_.push_back(regular_);
        exe_.push_back(df_);
        break;
      }
      kicked_flag_ = kicked(ball);
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        setplay_      = setplay();
        df_           = defense(agent::defense::defense_mode::normal_mode, false);
        regular_flag_ = true;
        kicked_flag_  = false;
      }
      exe_.push_back(setplay_);
      exe_.push_back(df_);
      break;

    case command::setplay_defense:
      //定常状態に入る
      if (kicked_flag_) {
        if (regular_flag_) {
          regular_      = regular(true);
          df_           = defense(agent::defense::defense_mode::normal_mode, false);
          regular_flag_ = false;
        }
        exe_.push_back(regular_);
        exe_.push_back(df_);
        break;
      }

      kicked_flag_ = kicked(ball);
      if (ball.x() < -3500) {       //コーナーキックの場合はマークを使う
        if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
          df_           = defense(agent::defense::defense_mode::normal_mode, true);
          regular_flag_ = true;
          kicked_flag_  = false;
        }
        exe_.push_back(df_);
      } else {
        if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
          regular_      = regular(false);
          df_           = defense(agent::defense::defense_mode::normal_mode, false);
          regular_flag_ = true;
          kicked_flag_  = false;
        }
        exe_.push_back(regular_);
        exe_.push_back(df_);
      }
      break;

    default:
      halt_ = halt();
      exe_.push_back(halt_);
      break;
  }

  previous_ball_       = ball;
  previous_command_    = current_command_;
  previous_refcommand_ = command;

  return exe_;
}

//ボールを蹴ったか判定
bool first_formation::kicked(model::ball ball) {
  if (std::hypot(ball.x() - previous_ball_.x(), ball.y() - previous_ball_.y()) > 80) {
    return true;
  }
  return false;
}

bool first_formation::is_command_changed() {
  return current_command_ != previous_command_;
}

///////////////////////////////////////////////////////
//              idを割り振る関数                       //
///////////////////////////////////////////////////////

// refboxからキーパーのidを得る
void first_formation::update_keeper() {
  if (is_yellow_) {
    keeper_ = refcommand_.team_yellow().goalie();
  } else {
    keeper_ = refcommand_.team_blue().goalie();
  }
}

//壁の数を決定する
void first_formation::decide_wall_count() {
  wall_count_ = ids_.size() < 4 ? 1 : 2;
}

//壁にするロボットを決定
void first_formation::decide_wall() {
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto keeper     = keeper_;
  auto tmp_ids          = ids_;
  //壁が見えているか判定 壁が見えていて,数が足りているとき作り直さない
  if (wall_count_ == wall_.size() &&
      std::all_of(wall_.begin(), wall_.end(),
                  [&our_robots](auto&& id) { return our_robots.count(id); })) {
    return;
  }

  if (std::all_of(ids_.begin(), ids_.end(),
                  [&our_robots](auto&& id) { return our_robots.count(id); })) {
    decide_wall_count();
    wall_.clear();

    //キーパーを候補から除外
    tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), keeper));

    // wall_countの値だけ前か順に壁にする
    auto wall_it = tmp_ids.begin();
    for (int i = 0; i < wall_count_; i++) {
      if (wall_it != tmp_ids.end()) {
        wall_.push_back(*wall_it);
      }
      wall_it++;
    }
    other_robots();
  }
}

//キッカーを決定
void first_formation::decide_kicker() {
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();
  auto tmp_ids          = others_; //キーパー,壁以外のロボットを抽出

  if (std::all_of(tmp_ids.begin(), tmp_ids.end(),
                  [&our_robots](auto&& id) { return our_robots.count(id); })) {
    //ボールに近いロボットをキッカーに決定
    const auto kicker_it = std::min_element(
        tmp_ids.cbegin(), tmp_ids.cend(), [&ball, &our_robots](auto& a, auto& b) {
          return std::hypot(our_robots.at(a).x() - ball.x(), our_robots.at(a).y() - ball.y()) <
                 std::hypot(our_robots.at(b).x() - ball.x(), our_robots.at(b).y() - ball.y());
        });

    kicker_ = *kicker_it;
    return;
  }
  kicker_ = 0;
}

//キーパー,壁以外のロボットを抽出
void first_formation::other_robots() {
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto keeper     = keeper_;
  const auto wall       = wall_;
  auto tmp_ids          = ids_;

  if (std::none_of(ids_.begin(), ids_.end(),
                   [&our_robots](auto&& id) { return our_robots.count(id); })) {
    kicker_ = 0;
    return;
  }

  //キーパー,壁を除外
  tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), keeper));
  tmp_ids.erase(
      std::remove_if(tmp_ids.begin(), tmp_ids.end(),
                     [&wall](auto&& id) { return std::count(wall.begin(), wall.end(), id); }),
      tmp_ids.end());
  others_ = tmp_ids;
}

//キーパー,壁,キッカー以外のロボットを抽出
void first_formation::waiter() {
  auto tmp_ids = others_;
  auto kicker  = kicker_;
  tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), kicker));
  waiter_ = tmp_ids;
}

//キーパー以外のロボットを抽出
void first_formation::except_keeper() {
  auto tmp_ids      = ids_;
  const auto keeper = keeper_;
  tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), keeper));
  except_keeper_ = tmp_ids;
}

///////////////////////////////////////////////////////
//              agentを呼び出す関数                    //
///////////////////////////////////////////////////////
std::shared_ptr<agent::halt> first_formation::halt() {
  return std::make_shared<agent::halt>(world_, is_yellow_, ids_);
}

std::shared_ptr<agent::stopgame> first_formation::stop() {
  return std::make_shared<agent::stopgame>(world_, is_yellow_, others_);
}

//引数でディフェンスモード,マークの有無を指定
std::shared_ptr<agent::defense> first_formation::defense(agent::defense::defense_mode mode,
                                                         bool mark_flag) {
  std::shared_ptr<agent::defense> df;
  std::vector<unsigned int> dummy;
  // normalかpkか判定
  if (mode == agent::defense::defense_mode::normal_mode) {
    if (mark_flag) {
      //壁を1台以下にして,あとはマークにつく
      for (auto it = wall_.begin(); wall_.size() > 1; it++) {
        others_.push_back(*it);
        wall_.erase(it);
      }
      df = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, wall_, others_);

    } else {
      df = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, wall_, dummy);
    }
  } else {
    df = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, dummy, dummy);
  }
  df->set_mode(mode);
  return df;
}

//引数でstart_flag,攻撃,守備を指定
std::shared_ptr<agent::penalty_kick> first_formation::pk(bool start_flag, bool attack) {
  std::shared_ptr<agent::penalty_kick> pk;
  if (!start_flag) { // normal_startが入ったときは初期化しない
    if (attack) {
      pk = std::make_shared<agent::penalty_kick>(world_, is_yellow_, kicker_, waiter_);
      pk->set_mode(agent::penalty_kick::penalty_mode::attack);
    } else {
      pk = std::make_shared<agent::penalty_kick>(world_, is_yellow_, kicker_, except_keeper_);
      pk->set_mode(agent::penalty_kick::penalty_mode::defense);
    }
  }

  pk->set_start_flag(start_flag);
  return pk;
}

//引数で攻撃,守備を指定
std::shared_ptr<agent::kick_off_waiter> first_formation::kickoff_waiter(
    agent::kick_off_waiter::kickoff_mode mode, bool attack) {
  std::shared_ptr<agent::kick_off_waiter> kickoff_waiter;
  //攻撃と守備でロボットの台数を変える
  if (attack) {
    kickoff_waiter = std::make_shared<agent::kick_off_waiter>(world_, is_yellow_, waiter_);
  } else {
    kickoff_waiter = std::make_shared<agent::kick_off_waiter>(world_, is_yellow_, others_);
  }
  kickoff_waiter->set_mode(mode);
  return kickoff_waiter;
}

//引数でstart_flagを指定
std::shared_ptr<agent::kick_off> first_formation::kickoff(bool start_flag) {
  std::shared_ptr<agent::kick_off> kickoff;
  if (!start_flag ||
      (start_flag && kickoff == nullptr)) { // normal_startが入ったときは初期化しない
    kickoff = std::make_shared<agent::kick_off>(world_, is_yellow_, kicker_);
  }
  kickoff->set_start_flag(start_flag);
  return kickoff;
}

std::shared_ptr<agent::setplay> first_formation::setplay() {
  return std::make_shared<agent::setplay>(world_, is_yellow_, kicker_, waiter_);
}

//引数でchase_ballを指定
std::shared_ptr<agent::regular> first_formation::regular(bool chase_flag) {
  std::shared_ptr<agent::regular> regular;
  regular = std::make_shared<agent::regular>(world_, is_yellow_, others_);
  regular->use_chaser(chase_flag);
  return regular;
}

//コマンドを使いやすい形に変換
first_formation::command first_formation::convert_command(model::refbox::game_command command) {
  using cmd = ai_server::model::refbox::game_command;

  switch (command) {
    case cmd::halt:
    case cmd::timeout_blue:
    case cmd::timeout_yellow:
      return command::halt;

    case cmd::normal_start:
      if (previous_refcommand_ == cmd::prepare_kickoff_yellow) {
        if (is_yellow_) {
          return command::kickoff_attack_start;
        } else {
          return command::kickoff_defense;
        }
      } else if (previous_refcommand_ == cmd::prepare_kickoff_blue) {
        if (!is_yellow_) {
          return command::kickoff_attack_start;
        } else {
          return command::kickoff_defense;
        }
      } else if (previous_refcommand_ == cmd::prepare_penalty_yellow) {
        if (is_yellow_) {
          return command::penalty_attack_start;
        } else {
          return command::penalty_defense;
        }
      } else if (previous_refcommand_ == cmd::prepare_penalty_blue) {
        if (!is_yellow_) {
          return command::penalty_attack_start;
        } else {
          return command::penalty_defense;
        }
      } else if (current_command_ == command::kickoff_attack_start) {
        return command::kickoff_attack_start;
      } else if (current_command_ == command::penalty_attack_start) {
        return command::penalty_attack_start;
      } else if (current_command_ == command::kickoff_defense) {
        return command::kickoff_defense;
      } else if (current_command_ == command::penalty_defense) {
        return command::penalty_defense;
      } else {
        return command::force_start;
      }

    case cmd::force_start:
      return command::force_start;

    case cmd::prepare_kickoff_yellow:
      if (is_yellow_)
        return command::kickoff_attack;
      else
        return command::kickoff_defense;

    case cmd::prepare_kickoff_blue:
      if (!is_yellow_)
        return command::kickoff_attack;
      else
        return command::kickoff_defense;

    case cmd::prepare_penalty_yellow:
      if (is_yellow_)
        return command::penalty_attack;
      else
        return command::penalty_defense;

    case cmd::prepare_penalty_blue:
      if (!is_yellow_)
        return command::penalty_attack;
      else
        return command::penalty_defense;
      break;

    case cmd::direct_free_yellow:
    case cmd::indirect_free_yellow:
      if (is_yellow_)
        return command::setplay_attack;
      else
        return command::setplay_defense;

    case cmd::direct_free_blue:
    case cmd::indirect_free_blue:
      if (!is_yellow_)
        return command::setplay_attack;
      else
        return command::setplay_defense;

    case cmd::stop:
    case cmd::ball_placement_blue:
    case cmd::ball_placement_yellow:
    case cmd::goal_blue:
    case cmd::goal_yellow:
    default:
      return command::stop;
  }
}
}
}
}
