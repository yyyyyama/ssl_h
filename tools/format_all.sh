#!/usr/bin/env bash

cd "$(dirname "$0")/.."

shopt -s nullglob globstar

for f in ./{app,src,test}/**/*.{cc,h}; do
  echo "formatting $f"
  clang-format -i "$f"
done
