#!/usr/bin/env bash
# Install Ollama to ~/.local (no sudo needed) and pull gemma3:4b.
# Run once before launching the drone_nl_control stack.
set -euo pipefail

OLLAMA_VERSION="v0.16.3"
INSTALL_DIR="$HOME/.local"
BIN_DIR="$INSTALL_DIR/bin"

echo "=== Downloading Ollama ${OLLAMA_VERSION} ==="
mkdir -p "$BIN_DIR"
TARBALL="/tmp/ollama-linux-amd64.tar.zst"
curl -L --progress-bar \
  "https://github.com/ollama/ollama/releases/download/${OLLAMA_VERSION}/ollama-linux-amd64.tar.zst" \
  -o "$TARBALL"

echo ""
echo "=== Extracting to ${INSTALL_DIR} ==="
tar --use-compress-program=zstd -xf "$TARBALL" -C "$INSTALL_DIR"
rm -f "$TARBALL"

echo ""
echo "=== Ensuring ${BIN_DIR} is in PATH ==="
if ! echo "$PATH" | grep -q "$BIN_DIR"; then
    echo "export PATH=\"${BIN_DIR}:\$PATH\"" >> "$HOME/.bashrc"
    export PATH="${BIN_DIR}:$PATH"
    echo "Added ${BIN_DIR} to PATH (restart shell or run: export PATH=${BIN_DIR}:\$PATH)"
fi
export PATH="${BIN_DIR}:$PATH"
export LD_LIBRARY_PATH="${INSTALL_DIR}/lib/ollama:${LD_LIBRARY_PATH:-}"

echo ""
echo "=== Ollama version ==="
ollama --version

echo ""
echo "=== Starting Ollama service ==="
if ! pgrep -x ollama >/dev/null 2>&1; then
    ollama serve &>/tmp/ollama.log &
    OLLAMA_PID=$!
    echo "Ollama started (PID $OLLAMA_PID), log at /tmp/ollama.log"
    sleep 3
else
    echo "Ollama already running"
fi

echo ""
echo "=== Pulling gemma3:4b model (~3.3 GB) ==="
ollama pull gemma3:4b

echo ""
echo "=== Smoke test ==="
ollama list

echo ""
echo "=== Done! ==="
echo "gemma3:4b is ready (running on GPU: $(ollama ps 2>/dev/null | grep gemma || echo 'check with: ollama ps'))"
echo ""
echo "To use in future shells, add to ~/.bashrc:"
echo "  export PATH=\"${BIN_DIR}:\$PATH\""
echo "  export LD_LIBRARY_PATH=\"${INSTALL_DIR}/lib/ollama:\$LD_LIBRARY_PATH\""
echo ""
echo "Launch the drone controller:"
echo "  ros2 launch drone_nl_control nl_drone.launch.py mission:='find the walking person'"
