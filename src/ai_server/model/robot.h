#ifndef AI_SERVER_MODEL_ROBOT_H
#define AI_SERVER_MODEL_ROBOT_H

#include <chrono>
#include <functional>
#include <optional>

namespace ai_server {
namespace model {

class robot {
public:
  /// 状態推定に使う関数オブジェクトの型
  using estimator_type = std::function<
      // 推定に失敗したら nullopt
      std::optional<robot>(
          // 推定関数が呼ばれたときのオブジェクト
          const robot&,
          // 指定したい時刻とのオフセット
          std::chrono::system_clock::duration)>;

private:
  double x_;
  double y_;
  double vx_;
  double vy_;
  double theta_;
  double omega_;
  double ax_;
  double ay_;
  double alpha_;

  estimator_type estimator_;

public:
  robot();
  robot(double x, double y, double theta);

  double x() const;
  double y() const;
  double vx() const;
  double vy() const;
  double theta() const;
  double omega() const;
  double ax() const;
  double ay() const;
  double alpha() const;

  void set_x(double x);
  void set_y(double y);
  void set_vx(double vx);
  void set_vy(double vy);
  void set_theta(double theta);
  void set_omega(double omega);
  void set_ax(double ax);
  void set_ay(double ay);
  void set_alpha(double alpha);

  /// 状態推定を行う関数オブジェクトが設定されているか
  bool has_estimator() const;

  /// 状態推定を行う関数オブジェクトを設定する
  void set_estimator(estimator_type f);

  /// 状態推定を行う関数オブジェクトを解除する
  void clear_estimator();

  /// t 経過したときの状態を推定する
  std::optional<robot> state_after(std::chrono::system_clock::duration t) const;
};
} // namespace model
} // namespace ai_server

#endif
