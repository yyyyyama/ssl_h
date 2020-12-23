#ifndef AI_SERVER_MODEL_BALL_H
#define AI_SERVER_MODEL_BALL_H

#include <chrono>
#include <functional>
#include <optional>

namespace ai_server {
namespace model {

class ball {
public:
  /// 状態推定に使う関数オブジェクトの型
  using estimator_type = std::function<
      // 推定に失敗したら nullopt
      std::optional<ball>(
          // 推定関数が呼ばれたときのオブジェクト
          const ball&,
          // 指定したい時刻とのオフセット
          std::chrono::system_clock::duration)>;

private:
  double x_;
  double y_;
  double z_;
  double vx_;
  double vy_;
  double ax_;
  double ay_;
  bool is_lost_;

  estimator_type estimator_;

public:
  ball();
  ball(double x, double y, double z);
  double x() const;
  double y() const;
  double z() const;
  double vx() const;
  double vy() const;
  double ax() const;
  double ay() const;
  bool is_lost() const;

  void set_x(double x);
  void set_y(double y);
  void set_z(double z);
  void set_vx(double vx);
  void set_vy(double vy);
  void set_ax(double ax);
  void set_ay(double ay);
  void set_is_lost(bool is_lost);

  /// 状態推定を行う関数オブジェクトが設定されているか
  bool has_estimator() const;

  /// 状態推定を行う関数オブジェクトを設定する
  void set_estimator(estimator_type f);

  /// 状態推定を行う関数オブジェクトを解除する
  void clear_estimator();

  /// t 経過したときの状態を推定する
  std::optional<ball> state_after(std::chrono::system_clock::duration t) const;
};
} // namespace model
} // namespace ai_server
#endif
