#!/usr/bin/env bash
# Host unit tests for the beacon's pure logic (MeshtasticProto.h).
# No hardware or toolchain needed — just a C++ compiler.
set -euo pipefail
cd "$(dirname "$0")"
CXX="${CXX:-c++}"
out="$(mktemp -d)/test_proto"
"$CXX" -std=c++17 -Wall -Wextra -O2 test_proto.cpp -o "$out"
"$out"
