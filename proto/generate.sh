#!/usr/bin/env bash
set -euo pipefail

proto_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Override ROO_PB_ROOT when roo_pb is not a sibling checkout.
roo_pb_root="${ROO_PB_ROOT:-${proto_dir}/../../roo_pb}"
python3 "${roo_pb_root}/tools/generate.py" -I "${proto_dir}" \
  --out "${proto_dir}/../src" comms.proto
clang-format --style=Google -i "${proto_dir}/../src/comms.pb.h"
