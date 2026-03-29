#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build-regression"
LOG_DIR="${ROOT_DIR}/artifacts/regression"
LOG_FILE="${LOG_DIR}/run.log"

mkdir -p "${LOG_DIR}"

if ! command -v cmake >/dev/null 2>&1; then
  echo "cmake not found; install cmake to run regression pipeline" | tee "${LOG_FILE}"
  exit 2
fi

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release | tee "${LOG_FILE}"
cmake --build "${BUILD_DIR}" --target drone_regression -j | tee -a "${LOG_FILE}"
"${BUILD_DIR}/drone_regression" | tee -a "${LOG_FILE}"

echo "Regression artifacts written to ${LOG_FILE}" | tee -a "${LOG_FILE}"
