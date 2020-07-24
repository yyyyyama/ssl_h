#include "first_formation.h"
#include "ai_server/util/math/distance.h"
#include "ai_server/util/math/to_vector.h"

namespace ai_server::game::formation {

first_formation::first_formation(context& ctx, const model::refbox& refcommand,
                                 const id_vec& ids)
    : base(ctx, refcommand),
      default_wall_count_(0),
      wall_count_(default_wall_count_),
      ids_(ids),
      kicked_flag_(false),
      initialize_flag_(false),
      previous_command_(game_situation::halt, false, false),
      current_command_(game_situation::halt, false, false),
      previous_ball_(util::math::position(world().ball())) {
  past_ball_.fill(util::math::position(world().ball()));
  const auto our_robots = model::our_robots(world(), team_color());
  std::copy_if(ids.cbegin(), ids.cend(), std::back_inserter(previous_visible_robots_),
               [&our_robots](auto id) { return our_robots.count(id); });
}

std::vector<std::shared_ptr<agent::base>> first_formation::execute() {
  current_command_ = detail::convert_command(team_color(), previous_command_, refcommand_);
  const auto [situation, is_attack, is_start] = current_command_;
  const auto enemy_keeper                     = team_color() == model::team_color::yellow
                                ? refcommand_.team_blue().goalie()
                                : refcommand_.team_yellow().goalie();
  const auto point = std::chrono::steady_clock::now();

  const auto our_robots = model::our_robots(world(), team_color());

  const Eigen::Vector2d ball = util::math::position(world().ball());
  for (std::size_t i = 0; i < past_ball_.size() - 1; ++i) {
    past_ball_[i] = past_ball_[i + 1];
  }
  past_ball_.back() = ball;

  std::vector<std::shared_ptr<agent::base>> exe;

  //見えているロボットのIDを取得する
  id_vec visible_robots;
  std::copy_if(ids_.cbegin(), ids_.cend(), std::back_inserter(visible_robots),
               [&our_robots](auto id) { return our_robots.count(id); });

  //前回と見えているロボットが違うとき,初期化フラグを立てる
  initialize_flag_ = previous_visible_robots_ != visible_robots;

  // stopgame時に見えている台数に応じて台数の割り振りを更新する
  if (situation == game_situation::stop) {
    const auto num = static_cast<unsigned int>(visible_robots.size());
    if (num <= 2)
      wall_count_ = 0; // 2台以下なら壁は無し
    else if (num <= 4)
      wall_count_ = std::min(static_cast<int>(default_wall_count_), 1);
    else
      wall_count_ = default_wall_count_;
  }

  // idの割り振りを行う
  role_reset(visible_robots);

  // コマンドが切り替わったときフラグを初期化する
  if (is_command_changed()) {
    kicked_flag_         = false;
    initialize_flag_     = false;
    change_command_time_ = point;
  }

  if (!kicked_flag_) {
    kicked_flag_ = kicked(ball);
    if (kicked_flag_) kicked_time_ = point;
  }

  // game_command_typeによってagentを使い分ける
  switch (situation) {
    case game_situation::halt:
      exe.emplace_back(halt());
      break;

    case game_situation::timeout:
      exe.emplace_back(alignment());
      break;

    case game_situation::stop:
      reset_agent();
      if (std::hypot(ball.x() - (world().field().x_min() + world().field().penalty_length()),
                     ball.y() - 0) < 200) {
        exe.emplace_back(pk(false, false, enemy_keeper));
        exe.emplace_back(defense(agent::defense::defense_mode::pk_normal_mode));
      } else {
        stop_ = make_agent<agent::stopgame>(except_keeper_);
        df_   = make_agent<agent::defense>(keeper_, wall_);
        exe.emplace_back(stop_);
        exe.emplace_back(df_);
      }
      break;

    case game_situation::ball_placement:
      exe.emplace_back(ball_placement(is_attack));
      break;

    case game_situation::kickoff:
      if (is_attack) {
        if (is_start) {
          //定常状態に入る
          if ((kickoff_ && kickoff_->finished()) || time_over(point, change_command_time_, 8)) {
            exe.emplace_back(all());
            break;
          }
          exe.emplace_back(kickoff(true));
          exe.emplace_back(defense(agent::defense::defense_mode::normal_mode));
        } else {
          exe.emplace_back(kickoff(false));
          exe.emplace_back(defense(agent::defense::defense_mode::normal_mode));
        }
      } else {
        //定常状態に入る
        if (kicked_flag_) {
          exe.emplace_back(all());
          break;
        }
        kicked_flag_ = kicked(ball);
        exe.emplace_back(kickoff_waiter(agent::kick_off_waiter::kickoff_mode::defense, false));
        exe.emplace_back(defense(agent::defense::defense_mode::normal_mode));
      }
      break;

    case game_situation::penalty:
      if (is_attack) {
        //定常状態に入る
        if (is_start &&
            ((pk_ && pk_->finished()) || time_over(point, change_command_time_, 10))) {
          exe.emplace_back(all());
        } else {
          exe.emplace_back(pk(is_start, true, enemy_keeper));
          exe.emplace_back(defense(agent::defense::defense_mode::normal_mode));
        }
      } else {
        //定常状態に入る
        if (is_start && kicked_flag_ && time_over(point, kicked_time_, 1)) {
          exe.emplace_back(all());
        } else {
          exe.emplace_back(pk(false, false, enemy_keeper));
          exe.emplace_back(defense(agent::defense::defense_mode::pk_normal_mode));
        }
      }
      break;

    case game_situation::force_start:
      exe.emplace_back(all());
      break;

    case game_situation::setplay:
      if (is_attack) {
        //定常状態に入る
        if ((setplay_ && setplay_->finished()) || time_over(point, change_command_time_, 15)) {
          exe.emplace_back(all());
          break;
        }
        exe.emplace_back(setplay());
        exe.emplace_back(defense(agent::defense::defense_mode::normal_mode));
      } else {
        //定常状態に入る
        if (kicked_flag_) {
          if (time_over(point, kicked_time_, 1)) {
            exe.emplace_back(all());
            break;
          } else {
            stop_ = make_agent<agent::stopgame>(except_keeper_);
            df_   = make_agent<agent::defense>(keeper_, id_vec{});
            exe.emplace_back(stop_);
            exe.emplace_back(df_);
            break;
          }
        }
        kicked_flag_ = kicked(ball);
        if (kicked_flag_) {
          kicked_time_ = point;
        }
        stop_ = make_agent<agent::stopgame>(except_keeper_);
        df_   = make_agent<agent::defense>(keeper_, id_vec{});
        exe.emplace_back(stop_);
        exe.emplace_back(df_);
      }
      break;

    case game_situation::shootout:
      if (is_attack) {
        if (is_start) {
          exe.emplace_back(shootout(true, enemy_keeper));
        } else {
          exe.emplace_back(stop());
        }
      } else {
        if (is_start) {
          exe.emplace_back(pk(false, false, enemy_keeper));
          exe.emplace_back(defense(agent::defense::defense_mode::pk_extention_mode));
        } else {
          exe.emplace_back(pk(false, false, enemy_keeper));
          exe.emplace_back(defense(agent::defense::defense_mode::pk_normal_mode));
        }
      }
      break;

    default:
      exe.emplace_back(halt());
      break;
  }

  previous_command_        = current_command_;
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

bool first_formation::kicked(const Eigen::Vector2d& ball) {
  if (is_command_changed()) {
    previous_ball_ = ball;
    return false;
  } else {
    const auto& pb = previous_ball_;
    return std::all_of(past_ball_.cbegin(), past_ball_.cend(),
                       [&pb](auto&& a) { return util::math::distance(pb, a) > 200; });
  }
}

bool first_formation::is_command_changed() {
  return current_command_ != previous_command_;
}

bool first_formation::time_over(const std::chrono::steady_clock::time_point& point,
                                const std::chrono::steady_clock::time_point& time, int count) {
  return (point - time) > std::chrono::seconds(count);
}

void first_formation::reset_agent() {
  pk_.reset();
  halt_.reset();
  alignment_.reset();
  df_.reset();
  kickoff_.reset();
  kickoff_waiter_.reset();
  stop_.reset();
  all_.reset();
  setplay_.reset();
}

Eigen::Vector2d first_formation::abp_target() {
  return Eigen::Vector2d(std::get<0>(refcommand_.ball_placement_position()),
                         std::get<1>(refcommand_.ball_placement_position()));
}

///////////////////////////////////////////////////////
//              idを割り振る関数                     //
///////////////////////////////////////////////////////
void first_formation::update_keeper() {
  if (team_color() == model::team_color::yellow) {
    keeper_ = refcommand_.team_yellow().goalie();
  } else {
    keeper_ = refcommand_.team_blue().goalie();
  }
}

void first_formation::decide_wall(const id_vec& visible_robots) {
  //壁が見えているか判定 壁が見えていて,数が足りているとき作り直さない
  if (!initialize_flag_ && wall_count_ == wall_.size() &&
      std::all_of(wall_.cbegin(), wall_.cend(), [&visible_robots](auto&& id) {
        return std::any_of(visible_robots.cbegin(), visible_robots.cend(),
                           [&id](auto v) { return v == id; });
      })) {
    return;
  }

  wall_.clear();

  //キーパーを候補から除外
  auto tmp_ids = visible_robots;
  tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), keeper_), tmp_ids.end());

  // wall_countの値だけ前から順に壁にする
  std::copy_n(tmp_ids.cbegin(), std::min(wall_count_, tmp_ids.size()),
              std::back_inserter(wall_));
}

void first_formation::decide_kicker(const id_vec& visible_robots) {
  const auto our_robots = model::our_robots(world(), team_color());
  const auto ball       = world().ball();
  id_vec tmp_ids;
  std::copy_if(others_.cbegin(), others_.cend(), std::back_inserter(tmp_ids),
               [&our_robots](auto id) { return our_robots.count(id); });

  if (!tmp_ids.empty() &&
      std::all_of(tmp_ids.cbegin(), tmp_ids.cend(), [&visible_robots](auto&& id) {
        return std::find(visible_robots.cbegin(), visible_robots.cend(), id) !=
               visible_robots.cend();
      })) {
    // others_の中でボールに近いロボットをキッカーに決定
    const auto kicker_it = std::min_element(
        tmp_ids.cbegin(), tmp_ids.cend(), [&ball, &our_robots](auto a, auto b) {
          return util::math::distance(our_robots.at(a), ball) <
                 util::math::distance(our_robots.at(b), ball);
        });

    if (kicker_it != tmp_ids.cend()) {
      kicker_ = *kicker_it;
    }
  }
}

void first_formation::extract_other_robots(const id_vec& visible_robots) {
  id_vec tmp_ids;
  //見えているロボットからキーパー,壁を除外したものを抽出
  std::copy_if(visible_robots.cbegin(), visible_robots.cend(), std::back_inserter(tmp_ids),
               [this](auto id) {
                 return id != keeper_ && std::none_of(wall_.cbegin(), wall_.cend(),
                                                      [id](auto w) { return w == id; });
               });
  others_ = tmp_ids;
}

void first_formation::extract_waiter() {
  auto tmp_ids = others_;
  tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), kicker_), tmp_ids.end());
  tmp_ids.shrink_to_fit();
  waiter_ = tmp_ids;
}

void first_formation::extract_except_keeper(const id_vec& visible_robots) {
  auto tmp_ids = visible_robots;
  tmp_ids.erase(std::remove(tmp_ids.begin(), tmp_ids.end(), keeper_), tmp_ids.end());
  tmp_ids.shrink_to_fit();
  except_keeper_ = tmp_ids;
}

///////////////////////////////////////////////////////
//              agentを呼び出す関数                  //
///////////////////////////////////////////////////////
std::shared_ptr<agent::halt> first_formation::halt() {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!halt_ || is_command_changed() || initialize_flag_) {
    halt_ = make_agent<agent::halt>(ids_);
  }
  return halt_;
}

std::shared_ptr<agent::alignment> first_formation::alignment() {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!alignment_ || is_command_changed() || initialize_flag_) {
    alignment_ = make_agent<agent::alignment>(ids_);
  }
  return alignment_;
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
std::shared_ptr<agent::defense> first_formation::defense(agent::defense::defense_mode mode) {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!df_ || is_command_changed() || initialize_flag_) {
    // normalかpkか判定
    if (mode == agent::defense::defense_mode::normal_mode) {
      df_ = make_agent<agent::defense>(keeper_, wall_);

    } else if (mode == agent::defense::defense_mode::pk_normal_mode ||
               mode == agent::defense::defense_mode::pk_extention_mode) {
      df_ = make_agent<agent::defense>(keeper_, id_vec{});
    } else {
      df_ = make_agent<agent::defense>(keeper_, wall_);
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
    kickoff_waiter_ = make_agent<agent::kick_off_waiter>(attack ? waiter_ : others_);
    kickoff_waiter_->set_mode(mode);
  }
  return kickoff_waiter_;
}

//引数でstart_flagを指定
std::shared_ptr<agent::kick_off> first_formation::kickoff(bool start_flag) {
  // agentが空のとき,コマンドが変わったとき,見えるロボットが変わったときに初期化する
  if (!kickoff_ ||
      (!start_flag && is_command_changed())) { // normal_startが入ったときは初期化しない
    kickoff_ = make_agent<agent::kick_off>(kicker_, waiter_);
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

std::shared_ptr<agent::all> first_formation::all() {
  // agentが空のとき,コマンドが変わったときに初期化する
  if (!all_ || is_command_changed()) {
    all_ = make_agent<agent::all>(except_keeper_, keeper_);
  }
  return all_;
}

std::shared_ptr<agent::all> first_formation::shootout(
    bool start_flag, [[maybe_unused]] unsigned int enemy_keeper) {
  // agentが空のとき,コマンドが変わったときに初期化する
  if (!shoot_out_ ||
      (!start_flag && is_command_changed())) { // normal_startが入ったときは初期化しない
    id_vec kick;
    kick.push_back(shootout_id_);
    shoot_out_ = make_agent<agent::all>(kick, keeper_);
  }
  return shoot_out_;
}
} // namespace ai_server::game::formation
