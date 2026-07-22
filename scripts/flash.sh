#!/usr/bin/env bash

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${PROJECT_DIR}"

if [[ $# -ge 1 ]]; then
    idf.py -p "$1" flash
else
    idf.py flash
fi
