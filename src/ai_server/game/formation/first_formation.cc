#include "first_formation.h"

namespace ai_server {
namespace game {
namespace formation {

first_formation::first_formation(const model::world& world, const model::refbox& refcommand,
                                 bool is_yellow, const std::vector<unsigned int>& ids)
    : base(world, is_yellow),
      refcommand_(refcommand),
      keeper_(5),
      kicker_(0),
      ids_(ids),
      wall_number_(2),
      before_ball_(world.ball()),
      kicked_flag_(false) {}

std::vector<std::shared_ptr<agent::base>> first_formation::execute() {
  before_command_       = current_command_;
  current_command_      = command_make(refcommand_.command());
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto ball       = world_.ball();
  before_refcommand_    = refcommand_.command();
  exe_.clear();

  //壁を決定する,変更があった場合にはothers_も更新する
  wall_decide();
  //壁,キッカーなどの役割を決定
  if (current_command_ == command::stop && before_command_ != current_command_) {
    kicker_decide();
    except_keeper();
    waiter();
  }

  if (std::all_of(ids_.begin(), ids_.end(),
                  [&our_robots](auto&& id) { return our_robots.count(id); })) {
    // commandによってagentを使い分ける
    switch (current_command_) {
      case command::halt:
        halt();
        break;

      case command::stop:
        kicked_flag_ = false;
        stop();
        defense(agent::defense::defense_mode::normal_mode, false);
        break;

      case command::kickoff_attack_start:
        if (kickoff_->finished()) {
          regular(true);
          defense(agent::defense::defense_mode::normal_mode, false);
          break;
        }
        kickoff(true);
        kickoff_waiter(agent::kick_off_waiter::kickoff_mode::attack);
        defense(agent::defense::defense_mode::normal_mode, false);
        break;

      case command::penalty_attack_start:
        if (pk_->finished()) {
          regular(true);
          defense(agent::defense::defense_mode::normal_mode, false);
          break;
        }
        pk(true);
        defense(agent::defense::defense_mode::normal_mode, false);
        break;

      case command::force_start:
        regular(true);
        defense(agent::defense::defense_mode::normal_mode, false);
        break;

      case command::kickoff_attack:
        kickoff(false);
        kickoff_waiter(agent::kick_off_waiter::kickoff_mode::attack);
        defense(agent::defense::defense_mode::normal_mode, false);
        break;

      case command::kickoff_defense:
        if (kicked_flag_) {
          regular(true);
          defense(agent::defense::defense_mode::normal_mode, false);
          break;
        }
        kicked_flag_ = kicked();
        kickoff_waiter(agent::kick_off_waiter::kickoff_mode::defense);
        defense(agent::defense::defense_mode::normal_mode, false);
        break;

      case command::penalty_attack:
        pk(false);
        defense(agent::defense::defense_mode::normal_mode, false);
        break;

      case command::penalty_defense:
        if (kicked_flag_) {
          regular(true);
          defense(agent::defense::defense_mode::normal_mode, false);
          break;
        }
        kicked_flag_ = kicked();
        pk(false);
        defense(agent::defense::defense_mode::pk_mode, false);
        break;

      case command::setplay_attack:
        if (kicked_flag_) {
          regular(true);
          defense(agent::defense::defense_mode::normal_mode, false);
          break;
        }
        kicked_flag_ = kicked();
        setplay();
        defense(agent::defense::defense_mode::normal_mode, false);
        break;

      case command::setplay_defense:
        if (kicked_flag_) {
          regular(true);
          defense(agent::defense::defense_mode::normal_mode, false);
          break;
        }
        kicked_flag_ = kicked();
        if (ball.x() < -3500) { //コーナーキックの場合はマークを使う
          defense(agent::defense::defense_mode::normal_mode, true);

        } else {
          regular(false);
          defense(agent::defense::defense_mode::normal_mode, false);
        }
        break;

      default:
        halt();
        break;
    }
  }
  before_ball_ = world_.ball();
  return exe_;
}

//ボールを蹴ったか判定
bool first_formation::kicked() {
  const auto ball = world_.ball();
  std::cout << ball.x() << " : " << before_ball_.x() << std::endl;
  if (std::hypot(ball.x() - before_ball_.x(), ball.y() - before_ball_.y()) > 80) {
    before_ball_ = ball;
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////
//              idを割り振る関数                       //
///////////////////////////////////////////////////////

//壁の数を決定する
void first_formation::wall_number_decide() {
  wall_number_ = 2;

  if (ids_.size() < 4) {
    wall_number_ = 1;
  }
}

//壁にするロボットを決定
void first_formation::wall_decide() {
  //壁が見えているか判定
  for (auto id = wall_.begin(); id != wall_.end(); ++id) {
    const auto wall_it =
        std::find_if(ids_.begin(), ids_.end(), [id](auto& i) { return i == *id; });
    if (wall_it == ids_.end()) {
      break; //見えていないなら作る
    }
    if (id == wall_.end() - 1) {
      return; //見えていたら作らない
    }
  }
  const auto our_robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  const auto keeper     = keeper_;
  auto tmp_ids          = ids_;

  if (std::all_of(ids_.begin(), ids_.end(),
                  [&our_robots](auto&& id) { return our_robots.count(id); })) {
    wall_number_decide();
    wall_.clear();

    //キーパーを候補から除外
    tmp_ids.erase(std::find_if(tmp_ids.begin(), tmp_ids.end(),
                               [keeper](auto& i) { return i == keeper; }));

    // wall_numberの値だけ前か順に壁にする
    auto wall_it = tmp_ids.begin();
    for (int i = 0; i < wall_number_; i++) {
      if (wall_it != tmp_ids.end()) {
        wall_.push_back(*wall_it);
      }
      wall_it++;
    }
    other_robots();
  }
}

//キッカーを決定
void first_formation::kicker_decide() {
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
  auto tmp_ids          = ids_;

  if (std::none_of(ids_.begin(), ids_.end(),
                   [&our_robots](auto&& id) { return our_robots.count(id); })) {
    kicker_ = 0;
    return;
  }

  //キーパー,壁を除外
  tmp_ids.erase(
      std::find_if(tmp_ids.begin(), tmp_ids.end(), [keeper](auto& i) { return i == keeper; }));
  for (auto id = wall_.begin(); id != wall_.end(); ++id) {
    tmp_ids.erase(
        std::find_if(tmp_ids.begin(), tmp_ids.end(), [id](auto& i) { return i == *id; }));
  }
  others_ = tmp_ids;
}

//キーパー,壁,キッカー以外のロボットを抽出
void first_formation::waiter() {
  auto tmp_ids = others_;
  auto kicker  = kicker_;
  tmp_ids.erase(
      std::find_if(tmp_ids.begin(), tmp_ids.end(), [kicker](auto& i) { return i == kicker; }));
  waiter_ = tmp_ids;
}

//キーパー以外のロボットを抽出
void first_formation::except_keeper() {
  auto ids_tmp      = ids_;
  const auto keeper = keeper_;
  ids_tmp.erase(
      std::find_if(ids_tmp.begin(), ids_tmp.end(), [keeper](auto& i) { return i == keeper; }));
  except_keeper_ = ids_tmp;
}

///////////////////////////////////////////////////////
//              agentを呼び出す関数                    //
///////////////////////////////////////////////////////
void first_formation::halt() {
  halt_ = std::make_shared<agent::halt>(world_, is_yellow_, ids_);
  exe_.push_back(halt_);
}

void first_formation::stop() {
  stop_ = std::make_shared<agent::stopgame>(world_, is_yellow_, others_);
  exe_.push_back(stop_);
}

void first_formation::defense(agent::defense::defense_mode mode, bool mark_flag) {
  std::vector<unsigned int> dummy;
  // normalかpkか判定
  if (mode == agent::defense::defense_mode::normal_mode)
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
  else
    df_ = std::make_shared<agent::defense>(world_, is_yellow_, keeper_, dummy, dummy);
  df_->set_mode(mode);
  exe_.push_back(df_);
};

void first_formation::pk_waiter() {
  if (current_command_ == command::penalty_attack ||
      current_command_ == command::penalty_attack_start) {
    pk_waiter_ = std::make_shared<agent::penalty_kick_waiter>(world_, is_yellow_, waiter_);
  } else {
    pk_waiter_ =
        std::make_shared<agent::penalty_kick_waiter>(world_, is_yellow_, except_keeper_);
  }
  exe_.push_back(pk_waiter_);
};

void first_formation::pk(bool start_flag) {
  if (current_command_ != before_command_) {
    if (current_command_ == command::penalty_defense) {
      pk_ = std::make_shared<agent::penalty_kick>(world_, is_yellow_, kicker_, except_keeper_);
      pk_->set_mode(agent::penalty_kick::penalty_mode::defense);
    } else if (current_command_ == command::penalty_attack) {
      pk_ = std::make_shared<agent::penalty_kick>(world_, is_yellow_, kicker_, waiter_);
      pk_->set_mode(agent::penalty_kick::penalty_mode::attack);
    }
  }

  pk_->set_start_flag(start_flag);
  exe_.push_back(pk_);
}

void first_formation::kickoff_waiter(agent::kick_off_waiter::kickoff_mode mode) {
  //攻撃と守備でロボットの台数を変える
  if (current_command_ == command::kickoff_attack ||
      current_command_ == command::kickoff_attack_start) {
    kickoff_waiter_ = std::make_shared<agent::kick_off_waiter>(world_, is_yellow_, waiter_);
  } else {
    kickoff_waiter_ = std::make_shared<agent::kick_off_waiter>(world_, is_yellow_, others_);
  }
  kickoff_waiter_->set_mode(mode);
  exe_.push_back(kickoff_waiter_);
}

void first_formation::kickoff(bool start_flag) {
  if (current_command_ != before_command_ &&
      current_command_ != command::kickoff_attack_start) {
    kickoff_ = std::make_shared<agent::kick_off>(world_, is_yellow_, kicker_);
  }
  kickoff_->set_start_flag(start_flag);
  exe_.push_back(kickoff_);
}

void first_formation::setplay() {
  setplay_ = std::make_shared<agent::setplay>(world_, is_yellow_, kicker_, waiter_);
  exe_.push_back(setplay_);
}

//引数でchase_ballを指定
void first_formation::regular(bool chase_flag) {
  regular_ = std::make_shared<agent::regular>(world_, is_yellow_, others_);
  regular_->set_ball_chase(chase_flag);
  exe_.push_back(regular_);
}

//コマンドを使いやすい形に変換
first_formation::command first_formation::command_make(model::refbox::game_command command) {
  using cmd = ai_server::model::refbox::game_command;

  switch (command) {
    case cmd::halt:
    case cmd::timeout_blue:
    case cmd::timeout_yellow:
      return command::halt;

    case cmd::normal_start:
      if (before_refcommand_ == cmd::prepare_kickoff_yellow) {
        if (is_yellow_) {
          return command::kickoff_attack_start;
        } else {
          return command::kickoff_defense;
        }
      } else if (before_refcommand_ == cmd::prepare_kickoff_blue) {
        if (!is_yellow_) {
          return command::kickoff_attack_start;
        } else {
          return command::kickoff_defense;
        }
      } else if (before_refcommand_ == cmd::prepare_penalty_yellow) {
        if (is_yellow_) {
          return command::penalty_attack_start;
        } else {
          return command::penalty_defense;
        }
      } else if (before_refcommand_ == cmd::prepare_penalty_blue) {
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
