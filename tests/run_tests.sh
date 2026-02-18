#!/usr/bin/env bash
set -euo pipefail

output=$(godot --headless --script tests/test_runner.gd 2>&1)
printf '%s\n' "$output"

if printf '%s\n' "$output" | grep -q "^Total: [0-9]* passed, 0 failed$"; then
    exit 0
else
    exit 1
fi
