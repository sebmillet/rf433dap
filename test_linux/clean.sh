#!/usr/bin/bash

set -euo pipefail

for d in [0-9][0-9]; do
    rm -f "${d}"/tmpout.txt
done

