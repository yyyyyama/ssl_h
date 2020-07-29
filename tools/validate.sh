#!/usr/bin/env bash

set -euo pipefail

esc=$(printf '\033')
boldrederror="${esc}[1;31merror${esc}[0m"

cd "$(dirname "$0")/.."

if ! git rev-parse --is-inside-work-tree > /dev/null 2>&1; then
  echo "$boldrederror: ${PWD} is not git repository"
  exit 1
fi

mapfile -t files < <(git ls-files -s | grep -v ^16 | cut -f2)

test_encoding() {
  local f=$1
  local r=0
  local out
  out=$(file -b "$f")
  if [[ $out != *"UTF-8 Unicode text"* && $out != *"ASCII text"* ]] || \
     [[ $out == *"extended-ASCII text"* ]]; then
    echo "$boldrederror: file is not utf-8 (without BOM) encoded"
    echo " --> $f"
    echo
    r=1
  fi
  if [[ $out == *"with CRLF line terminators"* ]]; then
    echo "$boldrederror: file has CRLF line terminators"
    echo " --> $f"
    echo
    r=1
  fi
  return $r
}

test_format() {
  local f=$1
  if [[ "$f" =~ \.(cc|h)$ ]]; then
    local out
    out=$(diff -u "$f" <(clang-format "$f") 2>&1)
    local r=$?
    if [ $r != 0 ]; then
      echo "$boldrederror: file is not formatted"
      echo " --> $f"
      tail -n +3 <<< "$out" | \
        sed  "s/^-.*$/${esc}[31m\0${esc}[0m/;s/^+.*$/${esc}[32m\0${esc}[0m/;s/^/  | /"
      echo "  = note: you can fix this error by 'clang-format -i $f'"
      echo
      return 1
    fi
  fi
}

test_include_guard() {
  local f=$1
  if [[ ! "$f" =~ ^(src|test)/.*\.h$ ]]; then
    return 0
  fi

  # ファイルパスから正しいインクルードガードの文字列を生成する
  local s1
  s1=$(sed -r 's/^test\//ai_server\/\0/; s/^src\///; s/[\/\.-]+/_/g; s/^.*$/\U&/' <<< "$f")

  # ファイルからインクルードガードを読み込む
  local s2
  s2=$(grep -Pzo '#ifndef\s+\K\b(\w+)\b(?=\s+#define\s+\b\1\b)' "$f" | tr -d '\0')

  if [ -z "$s2" ]; then
    echo "$boldrederror: wrong include guard"
    echo " --> $f"
    echo "  = note: expected include gurd string is '$s1'"
    echo
    return 1
  elif [ "$s2" != "$s1" ]; then
    echo "$boldrederror: wrong include guard"
    echo " --> $f"
    echo "  = note: expected include gurd string is '$s1' but file has '$s2'"
    echo
    return 1
  fi
}

errors=0
for f in "${files[@]}"; do
  if [ ! -e "$f" ]; then
    continue
  fi
  for t in {test_encoding,test_format,test_include_guard}; do
    if ! $t "$f"; then
      errors=$(( errors+1 ))
    fi
  done
done

echo "${#files[@]} files checked, $errors errors found."

[ "$errors" == 0 ]
