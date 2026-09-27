#!/bin/sh
# Build setupcore and run its suites. Usage: build_linux.sh [Debug]
set -e
here=$(cd "$(dirname "$0")" && pwd)
cmake -S "$here" -B "$here/build" -DCMAKE_BUILD_TYPE="${1:-Release}"
cmake --build "$here/build" -j
"$here/build/setupcore_tests"
"$here/build/setupcore_search_tests"
echo
echo "setupcore: $here/build/setupcore"
