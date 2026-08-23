#!/usr/bin/env bash
set -euo pipefail

preset="${1:-linux-gcc-debug}"
case "${preset}" in
  linux-gcc-debug|linux-gcc-release|linux-clang-asan) ;;
  *)
    echo "Unsupported preset: ${preset}" >&2
    exit 2
    ;;
esac

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_root}"
cmake --workflow --preset "${preset}"
