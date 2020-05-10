#ifndef AI_SERVER_GAME_NNABLA_H
#define AI_SERVER_GAME_NNABLA_H

#include <string>
#include <unordered_map>
#include <vector>

#include <nbla/context.hpp>

namespace ai_server::game {

class nnabla {
public:
  /// .nnp ファイルのパスとその属性 (常に CPU を使う場合 true) のペア
  using nnp_file_type = std::pair<std::string, bool>;

  /// @param backend    バックエンド (データ型なし) のリスト (NNabla のドキュメントも参照)
  /// @param device_id  バックエンドのデバイス ID (NNabla のドキュメントも参照)
  /// @param nnp_files  .nnp ファイルのパスとその属性の map
  ///
  /// backend に渡された文字列が正しいものかや、nnp_files に渡されたファイルパスが
  /// 存在するかなどのチェックは行っていないため、呼び出し前に確認すること
  nnabla(const std::vector<std::string>& backend, const std::string& device_id,
         const std::unordered_map<std::string, nnp_file_type>& nnp_files);

  /// @brief            .nnp ファイルのパスとそれに適した nbla::Context を取得する
  /// @param key        .nnp ファイルに紐づけた名前
  /// @param data_type  計算に使うデータ型 (NNabla のドキュメントも参照)
  /// @param cached     array_class に Cached なものを使うか
  std::pair<nbla::Context, std::string> nnp(const std::string& key,
                                            const std::string& data_type, bool cached) const;

private:
  std::vector<std::string> backend_;
  std::string device_id_;
  std::unordered_map<std::string, nnp_file_type> nnp_files_;
};

} // namespace ai_server::game

#endif // AI_SERVER_GAME_NNABLA_H
