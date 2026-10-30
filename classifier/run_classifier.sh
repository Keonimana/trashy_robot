#!/usr/bin/env bash
# Starts one detector. Both publish the same CommandPoint on tcp :5555, so run only one.
# Usage: classifier/run_classifier.sh [learned|hsv]
set -euo pipefail

classifier_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
detector="${1:-learned}"

case "$detector" in
    learned) exec python3 "$classifier_dir/naive_classifier/serial_classifier.py" ;;
    hsv)     exec "$classifier_dir/build/bin/classifier" ;;
    *)       echo "Usage: $0 [learned|hsv]" >&2; exit 2 ;;
esac
