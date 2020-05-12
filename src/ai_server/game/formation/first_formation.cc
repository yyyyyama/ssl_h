#include "first_formation.h"

#include <thread>

namespace ai_server {
namespace game {
namespace formation {

using id_vec = std::vector<unsigned int>;

first_formation::first_formation(context& ctx, const model::refbox& refcommand,
                                 const std::vector<unsigned int>& ids)
    : base(ctx, refcommand),
      wall_count_(2),
      ids_(ids),
      kicked_flag_(false),
      initialize_flag_(false),
      regular_flag_(true),
      is_corner_(true),
      previous_command_(command::halt),
      previous_ball_(world().ball()),
      past_ball_(
          {world().ball(), world().ball(), world().ball(), world().ball(), world().ball()}) {}

std::vector<std::shared_ptr<agent::base>> first_formation::execute() {
  const auto command      = refcommand_.command();
  current_command_        = convert_command(command);
  const auto enemy_keeper = team_color() == model::team_color::yellow
                                ? refcommand_.team_blue().goalie()
                                : refcommand_.team_yellow().goalie();
  const auto point = std::chrono::high_resolution_clock::now();

  const auto our_robots = model::our_robots(world(), team_color());
  const auto enemies    = model::enemy_robots(world(), team_color());
  id_vec enemies_id;
  for (auto& enemy : enemies) {
    enemies_id.push_back(enemy.first);
  }

  const auto ball = world().ball();
  past_ball_.erase(past_ball_.begin());
  past_ball_.push_back(ball);

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
  role_reset(visible_robots);

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
        reset_agent();
      }
      if (std::hypot((ball.x() + 3500), (ball.y() - 0)) < 200) {
        exe.emplace_back(pk(false, false, enemy_keeper));
        exe.emplace_back(defense(agent::defense::defense_mode::pk_normal_mode, 0));
      } else {
        exe.emplace_back(stop());
        exe.emplace_back(defense(agent::defense::defense_mode::stop_mode, 0));
      }
      break;

    case command::ball_placement:
      if (is_command_changed() || prev_abp_target_ != abp_target()) {
        //コマンドが変わったタイミングでフラグを初期化しておく
        // palcementのtargetが変わったら初期化し直す
        kicked_flag_     = false;
        initialize_flag_ = false;
        regular_flag_    = true;
        prev_abp_target_ = abp_target();
        reset_agent();
        ball_placement_ = make_agent<agent::ball_placement>(ids_, abp_target(), true);
      }
      exe.emplace_back(ball_placement_);
      break;

    case command::ball_placement_enemy:
      if (is_command_changed()) { //コマンドが変わったタイミングでフラグを初期化しておく
        kicked_flag_     = false;
        initialize_flag_ = false;
        regular_flag_    = true;
        prev_abp_target_ = abp_target();
        reset_agent();
      }
      exe.emplace_back(ball_placement(false));
      break;

    case command::kickoff_attack_start:
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_flag_        = true;
        change_command_time_ = point;
      }
      //定常状態に入る
      if ((kickoff_ && kickoff_->finished()) || time_over(point, 8)) {
        //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
        if (regular_flag_ || initialize_flag_) {
          id_vec dummy;
          df_      = make_agent<agent::defense>(keeper_, wall_, dummy);
          regular_ = make_agent<agent::regular>(others_);
          regular_->use_chaser(true);
          regular_->customize_marking(agent::regular::mark_option::no_mark);
          regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
        }
        exe.emplace_back(regular_);
        exe.emplace_back(df_);
        break;
      }
      exe.emplace_back(kickoff(true));
      exe.emplace_back(kickoff_waiter(agent::kick_off_waiter::kickoff_mode::attack, true));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, 0));
      break;

    case command::penalty_attack_start:
      //定常状態に入る
      if (pk_ && pk_->finished()) {
        //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
        if (regular_flag_ || initialize_flag_) {
          id_vec dummy;
          df_      = make_agent<agent::defense>(keeper_, wall_, dummy);
          regular_ = make_agent<agent::regular>(others_);
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
      exe.emplace_back(pk(true, true, enemy_keeper));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, 0));
      break;

    case command::force_start:
      exe.emplace_back(regular(true));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, 0));
      break;

    case command::kickoff_attack:
      exe.emplace_back(kickoff(false));
      exe.emplace_back(kickoff_waiter(agent::kick_off_waiter::kickoff_mode::attack, true));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, 0));
      break;

    case command::kickoff_defense:
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_flag_        = true;
        kicked_flag_         = false;
        change_command_time_ = point;
      }
      //定常状態に入る
      if (kicked_flag_) {
        //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
        if (regular_flag_ || initialize_flag_) {
          id_vec dummy;
          df_      = make_agent<agent::defense>(keeper_, wall_, dummy);
          regular_ = make_agent<agent::regular>(others_);
          regular_->use_chaser(true);
          regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
        }
        exe.emplace_back(regular_);
        exe.emplace_back(df_);
        break;
      }

      kicked_flag_ = kicked(ball);
      exe.emplace_back(kickoff_waiter(agent::kick_off_waiter::kickoff_mode::defense, false));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, 0));
      break;

    case command::penalty_attack:
      exe.emplace_back(pk(false, true, enemy_keeper));
      exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, 0));
      break;

    case command::penalty_defense:
      //定常状態に入る
      if (kicked_flag_) {
        if (time_over(point, 3)) {
          //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
          if (regular_flag_ || initialize_flag_) {
            id_vec dummy;
            df_      = make_agent<agent::defense>(keeper_, wall_, dummy);
            regular_ = make_agent<agent::regular>(others_);
            regular_->use_chaser(true);
            regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
          }
          exe.emplace_back(regular_);
          exe.emplace_back(df_);
          break;
        } else {
          exe.emplace_back(pk(false, false, enemy_keeper));
          exe.emplace_back(defense(agent::defense::defense_mode::pk_normal_mode, 0));
          break;
        }
      }

      kicked_flag_ = kicked(ball);
      if (kicked_flag_) {
        change_command_time_ = point;
      }
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_flag_ = true;
        kicked_flag_  = false;
      }
      exe.emplace_back(pk(false, false, enemy_keeper));
      exe.emplace_back(defense(agent::defense::defense_mode::pk_normal_mode, 0));
      break;

    case command::setplay_attack:
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        change_command_time_ = point;
        is_corner_           = ball.x() > 2500;
      }
      ////定常状態に入る
      if ((setplay_ && setplay_->finished()) || time_over(point, 15)) {
        //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
        if (regular_flag_ || initialize_flag_) {
          id_vec dummy;
          df_      = make_agent<agent::defense>(keeper_, wall_, dummy);
          regular_ = make_agent<agent::regular>(others_);
          regular_->use_chaser(true);
          regular_flag_ = false; //一度だけ初期化するように,regular_flag_をfalseにしておく
        }
        exe.emplace_back(regular_);
        exe.emplace_back(df_);
        break;
      }

      if (is_corner_) {
        if (kicked_flag_) {
          id_vec wall = setplay_->free_robots();
          id_vec dummy;
          wall.erase(std::remove(wall.begin(), wall.end(), kicker_), wall.end());
          df_ = make_agent<agent::defense>(keeper_, wall, dummy);
          exe.emplace_back(setplay_);
          exe.emplace_back(df_);
        } else {
          if (is_command_changed()) {
            id_vec dummy;
            df_ = make_agent<agent::defense>(keeper_, dummy, dummy);
            std::copy(wall_.begin(), wall_.end(), std::back_inserter(waiter_));
            setplay_ = make_agent<agent::setplay>(kicker_, waiter_);
          }
          exe.emplace_back(setplay_);
          exe.emplace_back(df_);
        }
      } else {
        exe.emplace_back(setplay());
        exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, 0));
      }
      kicked_flag_ = kicked(ball);
      break;

    case command::setplay_defense:
      if (is_command_changed()) { //コマンドが切り替わった時だけ初期化
        regular_flag_ = true;
        kicked_flag_  = false;
        is_corner_    = ball.x() < -3000;
      }
      //定常状態に入る
      if (kicked_flag_) {
        if (is_corner_) {
          if (time_over(point, 3)) {
            //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
            if (regular_flag_ && !initialize_flag_) {
              id_vec dummy;
              id_vec wall;
              id_vec marking = df_->marking_ids();

              std::copy_n(marking.begin(), std::min(wall_count_, marking.size()),
                          std::back_inserter(wall));

              df_ = make_agent<agent::defense>(keeper_, wall, dummy);

              auto tmp_ids = visible_robots;
              tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), keeper_),
                            tmp_ids.end());
              tmp_ids.erase(std::remove_if(tmp_ids.begin(), tmp_ids.end(),
                                           [&wall](auto&& id) {
                                             return std::count(wall.begin(), wall.end(), id);
                                           }),
                            tmp_ids.end());

              regular_ = make_agent<agent::regular>(tmp_ids);
              regular_->use_chaser(true);
              regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
            } else if (initialize_flag_) {
              id_vec dummy;
              df_      = make_agent<agent::defense>(keeper_, wall_, dummy);
              regular_ = make_agent<agent::regular>(others_);
              regular_->use_chaser(true);
              regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
            }
            exe.emplace_back(regular_);
            exe.emplace_back(df_);
          } else {
            exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, 1));
          }
        } else {
          //定常状態に入った時に初期化,agentを初期化する関数群では対応できないため直接初期化する
          if (regular_flag_ || initialize_flag_) {
            id_vec dummy;
            df_      = make_agent<agent::defense>(keeper_, wall_, dummy);
            regular_ = make_agent<agent::regular>(others_);
            regular_->use_chaser(true);
            regular_flag_ = false; //一度だけ初期化するように,egular_flag_をfalseにしておく
          }
          exe.emplace_back(regular_);
          exe.emplace_back(df_);
        }
        break;
      }
      kicked_flag_ = kicked(ball);
      if (is_corner_) { //コーナーキックの場合はマークを使う
        exe.emplace_back(defense(agent::defense::defense_mode::normal_mode, 1));
      } else {
        exe.emplace_back(regular(false));
        exe.emplace_back(defense(agent::defense::defense_mode::stop_mode, 0));
      }
      if (kicked_flag_) {
        change_command_time_ = point;
      }
      break;

    case command::shootout_attack:
      exe.emplace_back(shootout(false, enemy_keeper));
      break;

    case command::shootout_attack_start:
      exe.emplace_back(shootout(true, enemy_keeper));
      break;

    case command::shootout_defense:
      exe.emplace_back(pk(false, false, enemy_keeper));
      exe.emplace_back(defense(agent::defense::defense_mode::pk_normal_mode, 0));
      break;

    case command::shootout_defense_start:
      exe.emplace_back(pk(false, false, enemy_keeper));
      exe.emplace_back(defense(agent::defense::defense_mode::pk_extention_mode, 0));

    default:
      exe.emplace_back(halt());
      break;
  }

  previous_command_        = current_command_;
  previous_refcommand_     = command;
  previous_visible_robots_ = visible_robots;
  initialize_flag_         = false;

  return exe;
}

void first_formation::role_reset(const id_vec& visible_robots) {
  update_keeper();
  decide_wall(visible_robots);
  extract_other_robots(visible_robots);
  decide_kicker(visible_robots);
  extract_except_keeper(visible_robots);
  extract_waiter();
}

bool first_formation::kicked(model::ball ball) {
  if (is_command_changed()) {
    previous_ball_ = ball;
  } else {
    auto& pb = previous_ball_;
    if (std::all_of(past_ball_.cbegin(), past_ball_.cend(), [&](auto&& a) {
          return std::hypot(pb.x() - a.x(), pb.y() - a.y()) > 100;
        })) {
      return true;
    }
  }
  return false;
}

bool first_formation::is_command_changed() {
  return current_command_ != previous_command_;
}

bool first_formation::time_over(std::chrono::high_resolution_clock::time_point point,
                                int count) {
  return (point - change_command_time_) > std::chrono::seconds(count);
}

void first_formation::reset_agent() {
  pk_.reset();
  halt_.reset();
  df_.reset();
  kickoff_.reset();
  kickoff_waiter_.reset();
  stop_.reset();
  ball_placement_.reset();
  regular_.reset();
  setplay_.reset();
}

Eigen::Vector2d first_formation::abp_target() const {
  return Eigen::Vector2d(std::get<0>(refcommand_.ball_placement_position()),
                         std::get<1>(refcommand_.ball_placement_position()));
}

///////////////////////////////////////////////////////
//              idを割り振る関数                       //
///////////////////////////////////////////////////////
void first_formation::update_keeper() {
  if (team_color() == model::team_color::yellow) {
    keeper_ = refcommand_.team_yellow().goalie();
  } else {
    keeper_ = refcommand_.team_blue().goalie();
  }
}

void first_formation::decide_wall_count(const id_vec& visible_robots) {
  wall_count_ = visible_robots.size() < 4 ? 1 : 2;
}

void first_formation::decide_wall(const id_vec& visible_robots) {
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

void first_formation::decide_kicker(const id_vec& visible_robots) {
  const auto our_robots = model::our_robots(world(), team_color());
  const auto ball       = world().ball();
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

void first_formation::extract_other_robots(const id_vec& visible_robots) {
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

void first_formation::extract_except_keeper(const id_vec& visible_robots) {
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
    halt_ = make_agent<agent::halt>(ids_);
  }
  return halt_;
}

//引数で敵味方どちらのabpか指定
std::shared_ptr<agent::ball_placement> first_formation::ball_placement(bool is_ally) {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったとき,targetが変わったときに初期化する
  if (!ball_placement_ || is_command_changed() || initialize_flag_ ||
      prev_abp_target_ != abp_target()) {
    ball_placement_  = make_agent<agent::ball_placement>(ids_, abp_target(), is_ally);
    prev_abp_target_ = abp_target();
  }
  return ball_placement_;
}

std::shared_ptr<agent::stopgame> first_formation::stop() {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!stop_ || is_command_changed() || initialize_flag_) {
    stop_ = make_agent<agent::stopgame>(others_);
  }
  return stop_;
}

//引数でディフェンスモード,マークの有無を指定
std::shared_ptr<agent::defense> first_formation::defense(agent::defense::defense_mode mode,
                                                         unsigned int mark_num) {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!df_ || is_command_changed() || initialize_flag_) {
    id_vec mark;
    // normalかpkか判定
    if (mode == agent::defense::defense_mode::normal_mode) {
      if (mark_num == 0) {
        df_ = make_agent<agent::defense>(keeper_, wall_, mark);
      } else {
        const auto our_robots = model::our_robots(world(), team_color());

        auto w = wall_;
        const auto wall_it =
            std::max_element(w.begin(), w.end(), [&our_robots](auto& a, auto& b) {
              return std::abs(our_robots.at(a).y()) < std::abs(our_robots.at(b).y());
            });
        if (wall_it != w.end()) {
          w.erase(wall_it);
          mark.push_back(*wall_it);
        }
        std::copy(w.begin(), w.end(), std::back_inserter(others_));
        df_ = make_agent<agent::defense>(keeper_, mark, others_);
      }
    } else if (mode == agent::defense::defense_mode::pk_normal_mode ||
               mode == agent::defense::defense_mode::pk_extention_mode) {
      df_ = make_agent<agent::defense>(keeper_, mark, mark);
    } else {
      df_ = make_agent<agent::defense>(keeper_, wall_, mark);
    }
    df_->set_mode(mode);
  }
  return df_;
}

//引数でstart_flag,攻撃,守備を指定
std::shared_ptr<agent::penalty_kick> first_formation::pk(bool start_flag, bool attack,
                                                         unsigned int enemy_keeper) {
  // agentが空のとき,コマンドが変わったときに初期化する
  if (!pk_ || (!start_flag && is_command_changed())) { // normal_startが入ったときは初期化しない
    if (attack) {
      pk_ = make_agent<agent::penalty_kick>(kicker_, waiter_, enemy_keeper);
      pk_->set_mode(agent::penalty_kick::penalty_mode::attack);
    } else {
      pk_ = make_agent<agent::penalty_kick>(kicker_, except_keeper_, enemy_keeper);
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
      kickoff_waiter_ = make_agent<agent::kick_off_waiter>(waiter_);
    } else {
      kickoff_waiter_ = make_agent<agent::kick_off_waiter>(others_);
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
    kickoff_ = make_agent<agent::kick_off>(kicker_);
  }
  kickoff_->set_start_flag(start_flag);
  return kickoff_;
}

std::shared_ptr<agent::setplay> first_formation::setplay() {
  // agentが空のとき,コマンドが変わったときに初期化する
  if (!setplay_ || is_command_changed()) {
    setplay_ = make_agent<agent::setplay>(kicker_, waiter_);
  }
  return setplay_;
}

//引数でchase_ballを指定
std::shared_ptr<agent::regular> first_formation::regular(bool chase_flag) {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!regular_ || is_command_changed() || initialize_flag_) {
    regular_ = make_agent<agent::regular>(others_);
    regular_->use_chaser(chase_flag);
  }
  return regular_;
}

std::shared_ptr<agent::penalty_kick> first_formation::shootout(bool start_flag,
                                                               unsigned int enemy_keeper) {
  // agentが空のとき,コマンドが変わったときに初期化する
  if (!shoot_out_ ||
      (!start_flag && is_command_changed())) { // normal_startが入ったときは初期化しない
    shoot_out_ = make_agent<agent::penalty_kick>(kicker_, waiter_, enemy_keeper);
    pk_->set_mode(agent::penalty_kick::penalty_mode::attack);
  }
  shoot_out_->set_start_flag(start_flag);
  return shoot_out_;
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
        if (team_color() == model::team_color::yellow) {
          return command::kickoff_attack_start;
        } else {
          return command::kickoff_defense;
        }
      } else if (previous_refcommand_ == cmd::prepare_kickoff_blue) {
        if (team_color() == model::team_color::blue) {
          return command::kickoff_attack_start;
        } else {
          return command::kickoff_defense;
        }
      } else if (previous_refcommand_ == cmd::prepare_penalty_yellow) {
        if (refcommand_.stage() == model::refbox::stage_name::penalty_shootout) {
          if (team_color() == model::team_color::yellow) {
            return command::shootout_attack_start;
          } else {
            return command::shootout_defense_start;
          }
        } else {
          if (team_color() == model::team_color::yellow) {
            return command::penalty_attack_start;
          } else {
            return command::penalty_defense;
          }
        }
      } else if (previous_refcommand_ == cmd::prepare_penalty_blue) {
        if (refcommand_.stage() == model::refbox::stage_name::penalty_shootout) {
          if (team_color() == model::team_color::blue) {
            return command::shootout_attack_start;
          } else {
            return command::shootout_defense_start;
          }
        } else {
          if (team_color() == model::team_color::blue) {
            return command::penalty_attack_start;
          } else {
            return command::penalty_defense;
          }
        }
      } else if (current_command_ == command::kickoff_attack_start) {
        return command::kickoff_attack_start;
      } else if (current_command_ == command::penalty_attack_start) {
        return command::penalty_attack_start;
      } else if (current_command_ == command::kickoff_defense) {
        return command::kickoff_defense;
      } else if (current_command_ == command::penalty_defense) {
        return command::penalty_defense;
      } else if (current_command_ == command::shootout_attack_start) {
        return command::shootout_attack_start;
      } else if (current_command_ == command::shootout_defense_start) {
        return command::shootout_defense_start;
      } else {
        return command::force_start;
      }
      break;

    case cmd::force_start:
      return command::force_start;
      break;

    case cmd::prepare_kickoff_yellow:
      if (team_color() == model::team_color::yellow)
        return command::kickoff_attack;
      else
        return command::kickoff_defense;
      break;

    case cmd::prepare_kickoff_blue:
      if (team_color() == model::team_color::blue)
        return command::kickoff_attack;
      else
        return command::kickoff_defense;
      break;

    case cmd::prepare_penalty_yellow:
      if (refcommand_.stage() == model::refbox::stage_name::penalty_shootout) {
        if (team_color() == model::team_color::yellow)
          return command::shootout_attack;
        else
          return command::shootout_defense;
      } else {
        if (team_color() == model::team_color::yellow)
          return command::penalty_attack;
        else
          return command::penalty_defense;
      }
      break;

    case cmd::prepare_penalty_blue:
      if (refcommand_.stage() == model::refbox::stage_name::penalty_shootout) {
        if (team_color() == model::team_color::blue)
          return command::shootout_attack;
        else
          return command::shootout_defense;
      } else {
        if (team_color() == model::team_color::blue)
          return command::penalty_attack;
        else
          return command::penalty_defense;
      }
      break;

    case cmd::direct_free_yellow:
    case cmd::indirect_free_yellow:
      if (team_color() == model::team_color::yellow)
        return command::setplay_attack;
      else
        return command::setplay_defense;

    case cmd::direct_free_blue:
    case cmd::indirect_free_blue:
      if (team_color() == model::team_color::blue)
        return command::setplay_attack;
      else
        return command::setplay_defense;

    case cmd::ball_placement_yellow:
      if (team_color() == model::team_color::yellow)
        return command::ball_placement;
      else
        return command::ball_placement_enemy;

    case cmd::ball_placement_blue:
      if (team_color() == model::team_color::blue)
        return command::ball_placement;
      else
        return command::ball_placement_enemy;

    case cmd::stop:
    case cmd::goal_blue:
    case cmd::goal_yellow:
      return command::stop;
    default:
      return command::stop;
  }
}
} // namespace formation
} // namespace game
} // namespace ai_server
