#!/bin/bash

BINDIR="$(cd "$(dirname "$0")" && pwd)/bin"

TESTS=(
    test_main
    test_rawimu
    test_gga
    test_gsv
    test_rmc
    test_gst
    test_gsa
    test_bestgnssposa
    test_dualantennaheadinga
    test_inspvaa
    test_insstdevsa
    test_rawimub
    test_bestgnssposb
    test_dualantennaheadingb
    test_inspvab
    test_insstdevsb
)

PASSED=0
FAILED=0

for t in "${TESTS[@]}"; do
    exe="$BINDIR/$t"
    if [[ ! -x "$exe" ]]; then
        echo "MISSING: $exe (run 'make' first)"
        ((FAILED++))
        continue
    fi
    if "$exe"; then
        ((PASSED++))
    else
        echo "FAILED:  $t (exit code $?)"
        ((FAILED++))
    fi
done

echo ""
echo "========================================"
echo "Results: $PASSED passed, $FAILED failed"
echo "========================================"

[[ $FAILED -eq 0 ]]
