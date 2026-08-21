#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_NAME="Unified_ESP32_2400_TX_via_UART"
BOARD_CONFIG_NAME="radiomaster.tx_2400.ranger-nano"
BUILD_DIR="${SCRIPT_DIR}/.pio/build/${ENV_NAME}"
OUTPUT_FILE="${SCRIPT_DIR}/elrsCombined.bin"
ESPTOOL_PY="${HOME}/.platformio/packages/tool-esptoolpy/esptool.py"
PLATFORMIO_PYTHON="${HOME}/.platformio/penv/bin/python"
PLATFORMIO="${HOME}/.platformio/penv/bin/platformio"

usage() {
  echo "Usage: $0"
  echo "Builds the Starbound Ranger Nano broadcast transmitter firmware."
}

if [[ $# -ne 0 ]]; then
  usage
  exit 1
fi

if [[ ! -x "${PLATFORMIO}" ]]; then
  echo "Error: PlatformIO was not found at ${PLATFORMIO}"
  exit 1
fi

if [[ ! -f "${ESPTOOL_PY}" ]]; then
  echo "Error: esptool.py was not found at ${ESPTOOL_PY}"
  echo "Build once with PlatformIO, then retry."
  exit 1
fi

if [[ ! -x "${PLATFORMIO_PYTHON}" ]]; then
  echo "Error: PlatformIO Python was not found at ${PLATFORMIO_PYTHON}"
  exit 1
fi

echo "Building ${ENV_NAME} with board config ${BOARD_CONFIG_NAME}..."
cd "${SCRIPT_DIR}"

BOARD_CONFIG="${BOARD_CONFIG_NAME}"

TMP_CONF="$(mktemp "${SCRIPT_DIR}/.platformio-board-config.XXXXXX.ini")"
trap 'rm -f "${TMP_CONF}"' EXIT

cat > "${TMP_CONF}" <<EOF
[platformio]
extra_configs = ${SCRIPT_DIR}/platformio.ini

[env:${ENV_NAME}]
board_config = ${BOARD_CONFIG}
EOF

PLATFORMIO_BUILD_FLAGS="-DSTARBOUND_RANGER ${PLATFORMIO_BUILD_FLAGS:-}" \
  "${PLATFORMIO}" run -c "${TMP_CONF}" -e "${ENV_NAME}"

echo "Merging output binaries into ${OUTPUT_FILE}..."
"${PLATFORMIO_PYTHON}" "${ESPTOOL_PY}" --chip esp32 merge_bin \
  -o "${OUTPUT_FILE}" \
  0x1000 "${BUILD_DIR}/bootloader.bin" \
  0x8000 "${BUILD_DIR}/partitions.bin" \
  0xE000 "${BUILD_DIR}/boot_app0.bin" \
  0x10000 "${BUILD_DIR}/firmware.bin"

if ! strings "${OUTPUT_FILE}" | grep -Fq "RadioMaster Ranger Nano 2.4GHz TX"; then
  echo "Error: combined image does not contain the Ranger Nano target configuration."
  exit 1
fi

echo "Done. Combined image ready: ${OUTPUT_FILE}"
echo "Flash offset: 0x0"
