#!/usr/bin/env bash
set -e
fc01_navigation_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$fc01_navigation_dir/navigation_environment.sh"
fc01_load_environment "$fc01_navigation_dir"
exec /usr/bin/python3 "$fc01_navigation_dir/navigation_service.py" "$@"
