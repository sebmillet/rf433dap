#!/usr/bin/bash

set -euo pipefail

PORT=/dev/ttyUSB0

PASSED=0
FAILED=0

cd ..

START=1
STOP=4
if [ -n "${1:-}" ]; then
    START="$1";
    STOP="$1";
fi

for ((i=START; i<=STOP; i++)); do

    echo "== ROUND $i"

    ./am rf433decode.ino -u --stty -t $i

    sleep 2

    if [ ${i} -eq 1 ] || [ ${i} -eq 2 ]; then
        cd test_linux/track
    else
        cd test_linux/decoder
    fi

    for d in [0-9][0-9]; do
        inpfile=$(ls "${d}"/code*)
        tmpout="${d}/tmpout${i}.txt"
        expfile="${d}/expect${i}.txt"
        ./exectest.sh "${inpfile}" "${tmpout}" "${PORT}"
        echo -n "$i:${d}"
        if cmp "${expfile}" "${tmpout}" > /dev/null 2> /dev/null; then
            PASSED=$((PASSED + 1))
            echo "    test ok"
        else
            FAILED=$((FAILED + 1))
            echo " ** TEST KO, actual output differs from expected"
        fi
    done

    cd ../..

done

echo "------"
echo "PASSED: ${PASSED}"
echo "FAILED: ${FAILED}"

if [ "${FAILED}" -eq 0 ]; then
    echo "OK"
else
    echo
    echo "**************"
    echo "***** KO *****"
    echo "**************"
    exit 1
fi
