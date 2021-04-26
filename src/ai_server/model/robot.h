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

  robot();
  robot(double x, double y, double theta);

  /// @brief x座標
  double x() const;
  /// @brief y座標
  double y() const;
  /// @brief ロボットの正面が向いている方向。x軸を基準とする。
  double theta() const;

  /// @brief x方向の速度
  double vx() const;
  /// @brief y方向の速度
  double vy() const;
  /// @brief 回転速度
  double omega() const;

  /// @brief x方向の加速度
  double ax() const;
  /// @brief y方向の加速度
  double ay() const;
  /// @brief 回転加速度
  double alpha() const;

  void set_x(double x);
  void set_y(double y);
  void set_theta(double theta);
  void set_vx(double vx);
  void set_vy(double vy);
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

private:
  double x_;
  double y_;
  double theta_;
  double vx_;
  double vy_;
  double omega_;
  double ax_;
  double ay_;
  double alpha_;

  estimator_type estimator_;
};
} // namespace model
} // namespace ai_server

#endif
