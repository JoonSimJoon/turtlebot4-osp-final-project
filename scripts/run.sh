#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
load_config
if [[ " $* " == *' --dry-run '* ]]; then
  exec python3 "$TB4_ROOT/scripts/session.py" run "$@"
fi
require_linux
source_workspace
exec /usr/bin/python3 "$TB4_ROOT/scripts/session.py" run "$@"
