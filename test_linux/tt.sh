#!/usr/bin/bash

set -euo pipefail

PORT=/dev/ttyUSB0

PASSED=0
FAILED=0

    # The -t option will cause 'am' script to set TESTPLAN macro in source code,
    # causing underlying defines to be set with appropriate values for the test
    # plan to be executed.
cd ..
./am rf433decode.ino -u --stty -t 1

sleep 2

cd test_linux/track

for d in [0-9][0-9]; do
    inpfile=$(ls "${d}"/code-*)
    tmpout="${d}"/tmpout.txt
    expfile="${d}"/expect.txt
    ./exectest.sh "${inpfile}" "${tmpout}" "${PORT}"
    echo -n "${d}"
    if cmp "${expfile}" "${tmpout}" > /dev/null; then
        PASSED=$((PASSED + 1))
        echo "    test ok"
    else
        FAILED=$((FAILED + 1))
        echo " ** TEST KO, actual output differs from expected"
    fi
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
