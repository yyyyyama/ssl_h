#ifndef AI_SERVER_PLANNER_DETAIL_CLIPPING_H
#define AI_SERVER_PLANNER_DETAIL_CLIPPING_H

#include <algorithm>
#include <type_traits>
#include <vector>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/segment.hpp>

namespace ai_server::planner::detail {

/// @brief クリッピングを行い，結果を返す
/// @param segment            クリッピング対象の線分
/// @param region             クリッピングする領域
/// @return クリッピング後の線分の集合
template <class Point, class Geometry>
inline auto clipped(const boost::geometry::model::segment<Point>& segment,
                    const Geometry& region) {
  using segment_type    = boost::geometry::model::segment<Point>;
  using linestring_type = boost::geometry::model::linestring<Point>;

  // clipping
  std::vector<linestring_type> clip;
  boost::geometry::intersection(segment, region, clip);

  // clipping前がsegmentなので，幾何学的性質から戻り値はsegmentの集合にできる．
  std::vector<segment_type> result;
  std::transform(clip.begin(), clip.end(), std::back_inserter(result), [](const auto& a) {
    return segment_type{a.front(), a.back()};
  });
  return result;
}
} // namespace ai_server::planner::detail

#endif
