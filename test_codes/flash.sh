#!/bin/bash

# --- Configuration ---
BOARD_FQBN="esp32:esp32:esp32"
BUILD_DIR="build"
PORT="COM2"
BAUD=921600
FLASH_OFFSET=0x0  # Arduino sketch default offset
ESPTOOL_EXE="esptool.exe"  # ⚠️ Update this path!

# --- Check Arguments ---
if [ "$#" -lt 1 ]; then
  echo "Usage: ./myesp.sh <sketch_folder>"
  exit 1
fi

PROJECT_NAME="$1"
BIN_NAME="${PROJECT_NAME}.ino.merged.bin"
BIN_PATH="${BUILD_DIR}/${BIN_NAME}"

# --- Step 1: Compile using arduino-cli ---
echo "[*] Compiling $PROJECT_NAME using arduino-cli..."
START_TIME=$(date +%s)

arduino-cli compile --fqbn "${BOARD_FQBN}" "${PROJECT_NAME}" --output-dir "${BUILD_DIR}"
if [ $? -ne 0 ]; then
  echo "[!] Compilation failed."
  exit 1
fi

if [ ! -f "${BIN_PATH}" ]; then
  echo "[!] Compiled binary not found: ${BIN_PATH}"
  exit 1
fi

END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))

echo "[✓] Compilation successful: ${BIN_PATH}"
echo "[⏱] Build time: ${ELAPSED} seconds"

# --- Step 2: Upload using esptool.exe ---
echo "[*] Uploading to ${PORT} via esptool.exe..."

"${ESPTOOL_EXE}" \
  --chip esp32 \
  --port "${PORT}" \
  --baud "${BAUD}" \
  --before default-reset \
  --after hard-reset \
  write-flash -z \
  "${FLASH_OFFSET}" \
  "${BIN_PATH}"

if [ $? -ne 0 ]; then
  echo "[!] Upload failed."
  exit 1
fi

echo "[✓] Upload complete."

# --- Step 3: Cleanup ---
echo "[*] Cleaning up build files..."
rm -rf "${BUILD_DIR}"
if [ $? -eq 0 ]; then
  echo "[✓] Removed build directory: ${BUILD_DIR}"
else
  echo "[!] Failed to remove build directory: ${BUILD_DIR}"
fi

echo "[✓] All operations completed successfully."
