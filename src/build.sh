#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_NAME="Unified_ESP32C3_2400_RX_via_UART"
FIRMWARE_NAME="Unified_ESP32C3_2400_RX"
BOARD_CONFIG_NAME="starbound-spectra"
BUILD_DIR="${SCRIPT_DIR}/.pio/build/${ENV_NAME}"
OUTPUT_FILE="${SCRIPT_DIR}/elrsCombined.bin"
ESPTOOL_PY="${HOME}/.platformio/packages/tool-esptoolpy/esptool.py"

usage() {
  echo "Usage: $0"
  echo "Builds the Starbound Spectra 2.4GHz RX profile by default."
}

if [[ $# -ne 0 ]]; then
  usage
  exit 1
fi

if ! command -v pio >/dev/null 2>&1; then
  echo "Error: PlatformIO command 'pio' not found in PATH."
  exit 1
fi

if [[ ! -f "${ESPTOOL_PY}" ]]; then
  echo "Error: esptool.py was not found at ${ESPTOOL_PY}"
  echo "Build once with PlatformIO, then retry."
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

pio run -c "${TMP_CONF}" -e "${ENV_NAME}"

echo "Merging output binaries into ${OUTPUT_FILE}..."
python3 "${ESPTOOL_PY}" --chip esp32c3 merge_bin \
  -o "${OUTPUT_FILE}" \
  0x0 "${BUILD_DIR}/bootloader.bin" \
  0x8000 "${BUILD_DIR}/partitions.bin" \
  0xE000 "${BUILD_DIR}/boot_app0.bin" \
  0x10000 "${BUILD_DIR}/firmware.bin"

echo "Done. Combined image ready: ${OUTPUT_FILE}"
echo "Flash offset: 0x0"
