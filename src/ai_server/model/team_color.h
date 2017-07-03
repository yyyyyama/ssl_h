#ifndef AI_SERVER_MODEL_TEAM_COLOR_H
#define AI_SERVER_MODEL_TEAM_COLOR_H

namespace ai_server {
namespace model {

/// チームカラーを表現する列挙型
/// チームカラーの違いを表現するために使われてきたis_yellow変数と互換性を持たせるために,
/// 基底の型をboolとしている
enum class team_color : bool {
  blue   = false, ///< 青チーム
  yellow = true,  ///< 黄チーム
};

} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_TEAM_COLOR_H
