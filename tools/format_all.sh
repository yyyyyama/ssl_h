#!/bin/bash

. "$(dirname "$0")/is_toplevel.sh"

for f in $(git ls-files | grep -E '*\.(cc|h)$'); do
  echo "formatting ${f}"
  clang-format -i "${f}"
done
