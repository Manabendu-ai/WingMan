#!/usr/bin/env bash
set -e
LLAMA="${LLAMA_BIN:-$HOME/llama.cpp/build/bin/llama-server}"
MODEL="${MODEL_PATH:-$HOME/models/Qwen3-VL-4B-Instruct-UD-Q4_K_XL.gguf}"
MMPROJ="${MMPROJ_PATH:-$HOME/models/mmproj-4B-F16.gguf}"
exec "$LLAMA" --model "$MODEL" --mmproj "$MMPROJ" \
  --alias qwen3-vl -ngl 99 --ctx-size 4096 \
  --host 127.0.0.1 --port 8080
