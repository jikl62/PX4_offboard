#!/usr/bin/env bash
set -e
fc01_navigation_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec /usr/bin/python3 "$fc01_navigation_dir/navigation.py" "$@"
