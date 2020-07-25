#ifndef AI_SERVER_PLANNER_IMPL_HUMAN_LIKE_H
#define AI_SERVER_PLANNER_IMPL_HUMAN_LIKE_H

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>
#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/math/constants/constants.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ai_server/util/math/geometry_traits.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/planner/detail/clipping.h"
#include "ai_server/planner/detail/collision.h"
#include "ai_server/planner/obstacle_list.h"

namespace ai_server::planner::impl {
using box_type     = boost::geometry::model::box<Eigen::Vector2d>;
using segment_type = boost::geometry::model::segment<Eigen::Vector2d>;

/// @brief rayを伸ばす方向のリストを作る
/// @param dir                   基準方向
/// @param dir_count             伸ばす方向の数
/// @return 作成したリスト
inline std::vector<Eigen::Vector2d> make_directions(const Eigen::Vector2d& dir, int dir_count) {
  // rayとrayの間の角度
  const double step = boost::math::constants::two_pi<double>() / dir_count;

  const int max     = dir_count / 2 + 1;
  const bool is_odd = dir_count % 2 > 0;

  std::vector<Eigen::Vector2d> result;
  for (int i = 0; i < max; ++i) {
    Eigen::Rotation2D<double> rot{i * step};
    // +側
    result.push_back(rot * dir);
    // -側
    if (i > 0 && (is_odd || i + 1 < max)) {
      result.push_back(rot.inverse() * dir);
    }
  }
  return result;
}

/// @brief rayを伸ばす長さのリストを作る
/// @param l_min                 最小長さ
/// @param l_max                 最大長さ
/// @param l_step                １段階分の長さ
/// @return 作成したリスト
inline std::vector<double> make_length_list(double l_min, double l_max, double l_step) {
  std::vector<double> result;
  for (double l = l_min; l < l_max; l += l_step) {
    result.push_back(l);
  }
  if (l_min <= l_max) result.push_back(l_max);
  return result;
}

/// @brief  Human-Likeアルゴリズムが使えないときの経路探索を行い，結果を返す
/// @param start                 初期位置
/// @param dirs                  make_directions(...)で作成したリスト
/// @param lengths               make_length_list(...)で作成したリスト
/// @param obstacles             障害物リスト
/// @param area                  移動可能範囲
/// @return 経路探索の結果
inline std::optional<Eigen::Vector2d> exit_position(const Eigen::Vector2d& start,
                                                    const std::vector<Eigen::Vector2d>& dirs,
                                                    const std::vector<double>& lengths,
                                                    const obstacle_list::tree_type& obstacles,
                                                    const box_type& area) {
  // 候補がないとき
  if (dirs.empty() || lengths.empty()) return std::nullopt;

  // 脱出先までの距離
  auto exit_length = lengths.end();
  // 脱出先の方向
  Eigen::Vector2d exit_dir;

  { // 脱出先を探索

    // 方向，最小長さ，最大長さ
    std::vector<std::tuple<Eigen::Vector2d, double, double>> rays;
    // エリア内を通る範囲を調べる
    for (auto dir : dirs) {
      // 調査範囲の候補
      const segment_type line{start, start + lengths.back() * dir};
      // エリア内を通る部分を抽出
      const auto inside_line = detail::clipped(line, area);

      if (!inside_line.empty()) {
        // 線分をboxでクリッピングすると，結果は１本以下の線分になる
        const auto& [search_start, search_end] = inside_line.front();
        double min_length                      = (start - search_start).norm();
        double max_length                      = (start - search_end).norm();

        // エリア内を通れるときはリストに追加
        if (min_length < lengths.back() && max_length >= lengths.front()) {
          rays.emplace_back(dir, min_length, max_length);
        }
      }
    }

    // 障害物に当たらない地点を探す
    for (exit_length = lengths.begin(); exit_length != lengths.end(); ++exit_length) {
      auto find = std::find_if(
          rays.begin(), rays.end(), [&start, &obstacles, length = *exit_length](const auto& a) {
            return // エリア内を通れるか
                length > std::get<1>(a) && length <= std::get<2>(a) &&
                // ぶつからないか
                !detail::is_collided((start + length * std::get<0>(a)).eval(), obstacles);
          });

      // 見つかった
      if (find != rays.end()) {
        exit_dir = std::get<0>(*find);
        break;
      }
    }
  }

  // 脱出先がみつからない
  if (exit_length == lengths.end()) return std::nullopt;

  // 脱出先からの距離リスト
  std::vector<double> line_lengths;
  {
    // 最大長さになりうる値を導いておく
    const auto max_length = std::partition_point(
        std::next(exit_length, 1), lengths.end(), [&start, &exit_dir, &area](const auto& a) {
          return boost::geometry::within((start + a * exit_dir).eval(), area);
        });
    // 脱出先までの距離を取り除く
    std::transform(std::next(exit_length, 1), max_length, std::back_inserter(line_lengths),
                   [offset = *exit_length](double l) { return l - offset; });
  }

  // 暫定の脱出先
  const Eigen::Vector2d exit_p = start + *exit_length * exit_dir;
  // 暫定の脱出先からさらに線分を伸ばし，障害物に当たるときの長さを求める
  const auto collied_l = detail::find_collided_length(line_lengths.begin(), line_lengths.end(),
                                                      exit_p, exit_dir, obstacles);

  return collied_l == line_lengths.begin()
             // 脱出先まで
             ? exit_p
             // 障害物にぶつかる直前まで
             : exit_p + *std::prev(collied_l, 1) * exit_dir;
}

/// @brief  何もできないときに進む先を返す
/// @param start                 初期位置
/// @param dir                   進みたい方向
/// @param length                進む距離
/// @param area                  移動可能範囲
/// @return 経路探索の結果
inline Eigen::Vector2d default_position(const Eigen::Vector2d& start,
                                        const Eigen::Vector2d& dir, double length,
                                        const box_type& area) {
  // フィールド内にいるか
  if (boost::geometry::within(start, area)) {
    return start + length * dir;
  }
  // エリアの中心へ
  const auto center = boost::geometry::return_centroid<Eigen::Vector2d>(area);
  return start + length * (center - start).normalized();
}

/// @brief  Human-Likeアルゴリズムを使って経路探索を行い，結果を返す
/// @param start                 初期位置
/// @param dirs                  make_directions(...)で作成したリスト
/// @param lengths               make_length_list(...)で作成したリスト
/// @param obstacles             障害物リスト
/// @param area                  移動可能範囲
/// @return 経路探索の結果
inline std::optional<Eigen::Vector2d> planned_position(
    const Eigen::Vector2d& start, const std::vector<Eigen::Vector2d>& dirs,
    const std::vector<double>& lengths, const obstacle_list::tree_type& obstacles,
    const box_type& area) {
  if ( // 障害物に当たっている
      detail::is_collided(start, obstacles) ||
      // 候補が無い
      dirs.empty() || lengths.empty()) {
    // 失敗
    return std::nullopt;
  }

  // 最大長さ
  const double max_length = lengths.back();
  // 遮るものが何も無いときに進める方向
  const Eigen::Vector2d to_target = max_length * dirs.front();
  // (目的地に最も近いray, rayの先端とゴール間の距離の二乗)
  std::optional<std::pair<Eigen::Vector2d, double>> nearest;

  //  あらゆる方向にrayを伸ばして調べる
  for (const auto& dir : dirs) {
    // rayを最大まで伸ばしても目的地に近くならないことが明白なら終了
    if (nearest && (to_target - max_length * dir).squaredNorm() >= std::get<1>(*nearest)) {
      break;
    }

    const double dot = to_target.dot(dir);
    // ノードの最小長さとする値
    const double l_min =
        nearest ? dot - std::sqrt(std::get<1>(*nearest) - (to_target - dot * dir).squaredNorm())
                : lengths.front();

    // 調査範囲の候補
    const segment_type line{start + l_min * dir, start + max_length * dir};
    // エリア内を通る部分を抽出
    const auto inside_line = detail::clipped(line, area);

    // エリア内に入っている部分が無い
    if (inside_line.empty()) continue;
    // 線分をboxでクリッピングすると，結果は１本以下の線分になる
    const auto& [search_start, search_end] = inside_line.front();

    // 探索範囲の最小値
    const auto first =
        std::lower_bound(lengths.begin(), lengths.end(), (search_start - start).norm());
    // 探索範囲の最大値
    const auto last = std::upper_bound(first, lengths.end(), (search_end - start).norm());

    // 障害物に当たる，最小の長さを取得する
    const auto find = detail::find_collided_length(first, last, start, dir, obstacles);

    // 衝突したときの長さが最小長さでない
    if (find != first) {
      // ray
      Eigen::Vector2d ray = *std::prev(find, 1) * dir;
      // ゴールとの距離
      double squared_d = (ray - to_target).squaredNorm();

      // 今までの候補より目的地に近い
      if (!nearest || squared_d < std::get<1>(*nearest)) {
        nearest = std::make_pair(ray, squared_d);
      }
    }
  }
  // 成功
  if (nearest.has_value()) return start + std::get<0>(*nearest);
  // 失敗
  return std::nullopt;
}
} // namespace ai_server::planner::impl

#endif
