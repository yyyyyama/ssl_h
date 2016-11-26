#!/bin/bash

. "$(dirname "$0")/../is_toplevel.sh"

for f in $(git ls-files | grep -E '*\.(cc|h)$'); do
  if ! clang-format "${f}" | diff "${f}" - > /dev/null 2>&1; then
    echo "Error: Invalid code formatting in ${f}"
    exit 1
  fi
done
