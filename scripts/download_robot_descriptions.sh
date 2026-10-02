#!/bin/bash
# ATDog descriptions are maintained locally; never overwrite them with the upstream zoo.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
TARGET_DIR="${1:-src/rl_sar_zoo}"
for robot in atdog atdog2 atdog3; do
    if [ ! -f "${PROJECT_ROOT}/${TARGET_DIR}/${robot}_description/package.xml" ]; then
        echo "Missing ${robot}_description: restore it from this project's source checkout." >&2
        exit 1
    fi
done
echo "Local ATDog descriptions are ready."
