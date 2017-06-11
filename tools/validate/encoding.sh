#!/bin/bash

. "$(dirname "$0")/../is_toplevel.sh"

RETVAL=0

for f in $(git ls-files | grep -E '*\.(cc|h|txt)$'); do
  out=$(file -b "${f}")
  if [[ $out != *"UTF-8 Unicode text"* && $out != *"ASCII text"* ]]; then
    echo "Error: ${f} is not utf-8 (without BOM) encoded" >&2
    RETVAL=1
  fi
done

exit $RETVAL
