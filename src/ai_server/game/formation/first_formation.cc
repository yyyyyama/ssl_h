#include "first_formation.h"

namespace ai_server {
namespace game {
namespace formation {

using id_vec = std::vector<unsigned int>;

first_formation::first_formation(const model::world& world, const model::refbox& refcommand,
                                 bool is_yellow, const id_vec& ids)
    : base(world, is_yellow, refcommand),
      wall_count_(2),
      ids_(ids),
      kicked_flag_(false),
      initialize_flag_(false),
      regular_flag_(true),
      previous_command_(command::halt),
      previous_ball_(world.ball()) {}

std::vector<std::shared_ptr<agent::base>> first_formation::execute() {
  const auto command = refcommand_.command();
  current_command_   = convert_command(command);

  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto enemies    = !is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  id_vec enemies_id;
  for (auto& enemy : enemies) {
    enemies_id.push_back(enemy.first);
  }

  const auto ball = world_.ball();

  std::vector<std::shared_ptr<agent::base>> exe;

  //見えているロボットのIDを取得する
  id_vec visible_robots;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_robots),
               [&our_robots](auto id) { return our_robots.count(id); });

  //全て見えない場合は何もしない
  if (visible_robots.empty()) {
    return exe;
  }

  //前回と見えているロボットが違うとき,初期化フラグを立てる
  if (previous_visible_robots_ != visible_robots) {
    initialize_flag_ = true;
  }

  // idの割り振りを行う
  update_keeper();
  decide_wall(visible_robots);
  extract_other_robots(visible_robots);
  decide_kicker(visible_robots);
  extract_except_keeper(visible_robots);
  extract_waiter();

  // commandによってagentを使い分ける
  switch (current_command_) {
    case command::halt:
      exe.emplace_back(halt());
      break;

    case command::stop:
      if (is_command_changed()) { //コマンドが変わったタイミングでフラグを初期化しておく
        kicked_flag_     = false;
        initialize_flag_ = false;
        regular_flag_    = true;
      }
      exe.emplace_back(stop());
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, false));
      break;

    case command::kickoff_attack_start:
      //定常状態に入る
      if (kickoff_ && kickoff_->finished()) {
        //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
        if (regular_flag_ || initialize_flag_) {
          id_vec dummy;
          df_ = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, wall_, dummy);
          regular_ = std::make_shared<agent::regular>(world_, is_yellow_, others_);
          regular_->use_chaser(true);
          regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
        }
        exe.emplace_back(regular_);
        exe.emplace_back(df_);
        break;
      }

      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_flag_ = true;
      }
      exe.emplace_back(kickoff(true));
      exe.emplace_back(kickoff_waiter(agent::kick_off_waiter::kickoff_mode::attack, true));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, false));
      break;

    case command::penalty_attack_start:
      //定常状態に入る
      if (pk_ && pk_->finished()) {
        //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
        if (regular_flag_ || initialize_flag_) {
          id_vec dummy;
          df_ = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, wall_, dummy);
          regular_ = std::make_shared<agent::regular>(world_, is_yellow_, others_);
          regular_->use_chaser(true);
          regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
        }
        exe.emplace_back(regular_);
        exe.emplace_back(df_);
        break;
      }

      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_flag_ = true;
      }
      exe.emplace_back(pk(true, true));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, false));
      break;

    case command::force_start:
      exe.emplace_back(regular(true));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, false));
      break;

    case command::kickoff_attack:
      exe.emplace_back(kickoff(false));
      exe.emplace_back(kickoff_waiter(agent::kick_off_waiter::kickoff_mode::attack, true));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, false));
      break;

    case command::kickoff_defense:
      //定常状態に入る
      if (kicked_flag_) {
        //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
        if (regular_flag_ || initialize_flag_) {
          id_vec dummy;
          df_ = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, wall_, dummy);
          regular_ = std::make_shared<agent::regular>(world_, is_yellow_, others_);
          regular_->use_chaser(true);
          regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
        }
        exe.emplace_back(regular_);
        exe.emplace_back(df_);
        break;
      }

      kicked_flag_ = kicked(ball);
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_flag_ = true;
        kicked_flag_  = false;
      }
      exe.emplace_back(kickoff_waiter(agent::kick_off_waiter::kickoff_mode::defense, false));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, false));
      break;

    case command::penalty_attack:
      exe.emplace_back(pk(false, true));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, false));
      break;

    case command::penalty_defense:
      //定常状態に入る
      if (kicked_flag_) {
        //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
        if (regular_flag_ || initialize_flag_) {
          id_vec dummy;
          df_ = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, wall_, dummy);
          regular_ = std::make_shared<agent::regular>(world_, is_yellow_, others_);
          regular_->use_chaser(true);
          regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
        }
        exe.emplace_back(regular_);
        exe.emplace_back(df_);
        break;
      }

      kicked_flag_ = kicked(ball);
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_flag_ = true;
        kicked_flag_  = false;
      }
      exe.emplace_back(pk(false, false));
      exe.emplace_back(defense(agent::defense::defense_mode::pk_normal_mode, false));
      break;

    case command::setplay_attack:
      ////定常状態に入る
      if (setplay_ && setplay_->finished()) {
        //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
        if (regular_flag_ || initialize_flag_) {
          id_vec dummy;
          df_ = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, wall_, dummy);
          regular_ = std::make_shared<agent::regular>(world_, is_yellow_, others_);
          regular_->use_chaser(true);
          regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
        }
        exe.emplace_back(regular_);
        exe.emplace_back(df_);
        break;
      }
      kicked_flag_ = kicked(ball);
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_flag_ = true;
      }
      exe.emplace_back(setplay());
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, false));
      break;

    case command::setplay_defense:
      //定常状態に入る
      if (kicked_flag_) {
        //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
        if (regular_flag_ || initialize_flag_) {
          id_vec dummy;
          df_ = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, wall_, dummy);
          regular_ = std::make_shared<agent::regular>(world_, is_yellow_, others_);
          regular_->use_chaser(true);
          regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
        }
        exe.emplace_back(regular_);
        exe.emplace_back(df_);
        break;
      }

      kicked_flag_ = kicked(ball);
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_flag_ = true;
        kicked_flag_  = false;
      }
      if (ball.x() < -3500) { //コーナーキックの場合はマークを使う
        exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, true));
      } else {
        exe.emplace_back(regular(false));
        exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, false));
      }
      break;

    default:
      exe.emplace_back(halt());
      break;
  }

  previous_ball_           = ball;
  previous_command_        = current_command_;
  previous_refcommand_     = command;
  previous_visible_robots_ = visible_robots;
  initialize_flag_         = false;

  return exe;
}

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

void first_formation::update_keeper() {
  if (is_yellow_) {
    keeper_ = refcommand_.team_yellow().goalie();
  } else {
    keeper_ = refcommand_.team_blue().goalie();
  }
}

void first_formation::decide_wall_count(id_vec visible_robots) {
  wall_count_ = visible_robots.size() < 4 ? 1 : 2;
}

void first_formation::decide_wall(id_vec visible_robots) {
  const auto keeper = keeper_;
  auto tmp_ids      = visible_robots;
  decide_wall_count(visible_robots);
  //壁が見えているか判定 壁が見えていて,数が足りているとき作り直さない
  if (!initialize_flag_ && wall_count_ == wall_.size() &&
      std::all_of(wall_.begin(), wall_.end(), [&tmp_ids](auto&& id) {
        return std::find(tmp_ids.begin(), tmp_ids.end(), id) != tmp_ids.end();
      })) {
    return;
  }

  wall_.clear();

  //キーパーを候補から除外
  tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), keeper), tmp_ids.end());

  // wall_countの値だけ前から順に壁にする
  std::copy_n(tmp_ids.begin(), std::min(wall_count_, tmp_ids.size()),
              std::back_inserter(wall_));
}

void first_formation::decide_kicker(id_vec visible_robots) {
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();
  auto tmp_ids          = others_; //キーパー,壁以外のロボットを抽出

  if (!tmp_ids.empty() &&
      std::all_of(tmp_ids.begin(), tmp_ids.end(), [&visible_robots](auto&& id) {
        return std::find(visible_robots.begin(), visible_robots.end(), id) !=
               visible_robots.end();
      })) {
    //ボールに近いロボットをキッカーに決定
    const auto kicker_it = std::min_element(
        tmp_ids.cbegin(), tmp_ids.cend(), [&ball, &our_robots](auto& a, auto& b) {
          return std::hypot(our_robots.at(a).x() - ball.x(), our_robots.at(a).y() - ball.y()) <
                 std::hypot(our_robots.at(b).x() - ball.x(), our_robots.at(b).y() - ball.y());
        });
    if (kicker_it != tmp_ids.cend()) {
      kicker_ = *kicker_it;
    }
  }
}

void first_formation::extract_other_robots(id_vec visible_robots) {
  const auto keeper = keeper_;
  const auto wall   = wall_;
  auto tmp_ids      = visible_robots;

  //キーパー,壁を除外
  tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), keeper), tmp_ids.end());

  tmp_ids.erase(
      std::remove_if(tmp_ids.begin(), tmp_ids.end(),
                     [&wall](auto&& id) { return std::count(wall.begin(), wall.end(), id); }),
      tmp_ids.end());
  others_ = tmp_ids;
}

void first_formation::extract_waiter() {
  auto tmp_ids = others_;
  auto kicker  = kicker_;
  tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), kicker), tmp_ids.end());
  waiter_ = tmp_ids;
}

void first_formation::extract_except_keeper(id_vec visible_robots) {
  auto tmp_ids      = visible_robots;
  const auto keeper = keeper_;
  tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), keeper), tmp_ids.end());
  except_keeper_ = tmp_ids;
}

///////////////////////////////////////////////////////
//              agentを呼び出す関数                    //
///////////////////////////////////////////////////////
std::shared_ptr<agent::halt> first_formation::halt() {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!halt_ || is_command_changed() || initialize_flag_) {
    halt_ = std::make_shared<agent::halt>(world_, is_yellow_, ids_);
  }
  return halt_;
}

std::shared_ptr<agent::stopgame> first_formation::stop() {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!stop_ || is_command_changed() || initialize_flag_) {
    stop_ = std::make_shared<agent::stopgame>(world_, is_yellow_, others_);
  }
  return stop_;
}

//引数でディフェンスモード,マークの有無を指定
std::shared_ptr<agent::defense> first_formation::defense(agent::defense::defense_mode mode,
                                                         bool mark_flag) {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!df_ || is_command_changed() || initialize_flag_) {
    id_vec dummy;
    // normalかpkか判定
    if (mode == agent::defense::defense_mode::normal_mode) {
      if (mark_flag) {
        //壁を1台以下にして,あとはマークにつく
        for (auto it = wall_.begin(); wall_.size() > 1; it++) {
          others_.push_back(*it);
          wall_.erase(it);
        }
        df_ = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, wall_, others_);

      } else {
        df_ = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, wall_, dummy);
      }
    } else {
      df_ = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, dummy, dummy);
    }
    df_->set_mode(mode);
  }

  return df_;
}

//引数でstart_flag,攻撃,守備を指定
std::shared_ptr<agent::penalty_kick> first_formation::pk(bool start_flag, bool attack) {
  // agentが空のとき,コマンドが変わったときに初期化する
  if (!pk_ || (!start_flag && is_command_changed())) { // normal_startが入ったときは初期化しない
    if (attack) {
      pk_ = std::make_shared<agent::penalty_kick>(world_, is_yellow_, kicker_, waiter_);
      pk_->set_mode(agent::penalty_kick::penalty_mode::attack);
    } else {
      pk_ = std::make_shared<agent::penalty_kick>(world_, is_yellow_, kicker_, except_keeper_);
      pk_->set_mode(agent::penalty_kick::penalty_mode::defense);
    }
  }

  pk_->set_start_flag(start_flag);
  return pk_;
}

//引数で攻撃,守備を指定
std::shared_ptr<agent::kick_off_waiter> first_formation::kickoff_waiter(
    agent::kick_off_waiter::kickoff_mode mode, bool attack) {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!kickoff_waiter_ || is_command_changed() || initialize_flag_) {
    //攻撃と守備でロボットの台数を変える
    if (attack) {
      kickoff_waiter_ = std::make_shared<agent::kick_off_waiter>(world_, is_yellow_, waiter_);
    } else {
      kickoff_waiter_ = std::make_shared<agent::kick_off_waiter>(world_, is_yellow_, others_);
    }
    kickoff_waiter_->set_mode(mode);
  }
  return kickoff_waiter_;
}

//引数でstart_flagを指定
std::shared_ptr<agent::kick_off> first_formation::kickoff(bool start_flag) {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!kickoff_ ||
      (!start_flag && is_command_changed())) { // normal_startが入ったときは初期化しない
    kickoff_ = std::make_shared<agent::kick_off>(world_, is_yellow_, kicker_);
  }
  kickoff_->set_start_flag(start_flag);
  return kickoff_;
}

std::shared_ptr<agent::setplay> first_formation::setplay() {
  // agentが空のとき,コマンドが変わったときに初期化する
  if (!setplay_ || is_command_changed()) {
    setplay_ = std::make_shared<agent::setplay>(world_, is_yellow_, kicker_, waiter_);
  }
  return setplay_;
}

//引数でchase_ballを指定
std::shared_ptr<agent::regular> first_formation::regular(bool chase_flag) {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!regular_ || is_command_changed() || initialize_flag_) {
    regular_ = std::make_shared<agent::regular>(world_, is_yellow_, others_);
    regular_->use_chaser(chase_flag);
  }
  return regular_;
}

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
