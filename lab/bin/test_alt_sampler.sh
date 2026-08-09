#!/usr/bin/env bash
# test_alt_sampler.sh — pins the altitude path against the lost-message token.
#
# The bug this exists for: `ros2 topic echo` prints "A message was lost!!!" on
# STDOUT. `2>/dev/null` never suppressed it, `head -1` captured it, `tr -d ' '`
# compacted it to Amessagewaslost!!!, and both reporting awks admitted it
# (they only rejected "NA" and "") and coerced it to numeric 0. The cell then
# reported a 0.00 m altitude sample. No aircraft was ever there -- every real
# sample in all five clean cells is >= 1.98 m -- so an instrumentation artifact
# was one step from being reported as vehicle behaviour.
#
# Case 0 reproduces the OLD pipeline and asserts it still gets the WRONG
# answer. Without it this suite could pass against code that never had the bug,
# and would not prove the fix is what does the work.
#
# ROS is NOT sourced here on purpose: every case is a pure text transform, so
# the suite runs anywhere and cannot be broken by a ROS environment.

set -u                       # not -e: each case reports rather than aborts
BIN="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$BIN/hz_numeric.sh"

pass=0; fail=0
ok()   { pass=$((pass+1)); printf '  ok   %s\n' "$1"; }
bad()  { fail=$((fail+1)); printf '  FAIL %s\n     expected [%s] got [%s]\n' "$1" "$2" "$3"; }
is()   { [ "$2" = "$3" ] && ok "$1" || bad "$1" "$2" "$3"; }

# The exact stdout `ros2 topic echo --once --field ...` produced on the r9.0
# cell: the DDS diagnostic, then the real value.
POISON='A message was lost!!!'
LOST_THEN_VALUE=$(printf '%s\n2.00\n' "$POISON")

echo "== hz_numeric.first_numeric =="

# Case 0 — the OLD pipeline, kept as proof the fix is load-bearing.
old=$(printf '%s\n' "$LOST_THEN_VALUE" | head -1 | tr -d ' \r')
is "old pipeline still yields the poison token (bug reproduces)" \
   "Amessagewaslost!!!" "$old"
is "and awk coerced that token to 0" \
   "0" "$(echo "$old" | awk '{print $1+0}')"

is "diagnostic line then value -> value"      "2.00" \
   "$(printf '%s\n' "$LOST_THEN_VALUE" | first_numeric)"
is "several diagnostics then value -> value"  "1.98" \
   "$(printf '%s\n%s\nWARNING: whatever\n1.98\n' "$POISON" "$POISON" | first_numeric)"
is "bare value passes through"                "2.0037" \
   "$(printf '2.0037\n' | first_numeric)"
is "negative accepted (z is signed)"          "-0.5" \
   "$(printf -- '-0.5\n' | first_numeric)"
is "exponent accepted"                        "3.2e-05" \
   "$(printf '3.2e-05\n' | first_numeric)"
is "value with trailing CR survives"          "2.00" \
   "$(printf '2.00\r\n' | first_numeric)"
is "diagnostic only -> empty (sampler writes NA)" "" \
   "$(printf '%s\n' "$POISON" | first_numeric)"
is "empty input -> empty"                     "" "$(printf '' | first_numeric)"
is "first of several values wins"             "2.00" \
   "$(printf '2.00\n2.50\n' | first_numeric)"

echo "== regex is defined once, not copied out of step =="
# The awks cannot source a shell function, so they carry the regex as -v NUM.
# That is the only duplication in the fix, and this is what stops it drifting.
# -F is required: the thing being searched for IS a regex, and without it grep
# would parse the char classes and anchors as a pattern and match nothing.
# `|| true` and not `|| echo 0`: grep -c already prints 0, it just exits 1.
n_sites=$(grep -c -F -- "-v NUM=\"$HZ_NUMERIC_RE\"" "$BIN/lab_cell.sh" || true)
is "both lab_cell.sh awks carry the hz_numeric.sh regex verbatim" "2" "$n_sites"
is "lab_cell.sh sources hz_numeric.sh" "1" \
   "$(grep -c 'source .*hz_numeric.sh' "$BIN/lab_cell.sh")"
is "lab_cell.sh no longer uses head -1 on the odom field" "0" \
   "$(grep -c 'position.z 2>/dev/null | head -1' "$BIN/lab_cell.sh")"

echo "== reporting awk rejects, counts, and does not coerce =="
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
CSV="$TMP/alt.csv"
cat > "$CSV" <<'CSVEOF'
1754500000,2.00
1754500010,NA
1754500020,Amessagewaslost!!!
1754500030,1.98
1754500040,
CSVEOF

# Same body as lab_cell.sh's summary awk, driven by the shared regex.
report() {
  awk -F, -v NUM="$HZ_NUMERIC_RE" \
      '$2=="NA"||$2==""{next}
       $2 !~ NUM {bad++; next}
       {n++; v=$2+0; s+=v; if(n==1||v<min)min=v; if(n==1||v>max)max=v}
       END{printf "n=%d min=%.2f max=%.2f mean=%.2f bad=%d\n",n,min,max,s/n,bad+0}' "$1"
}
is "poison excluded; mean is of the two real samples only" \
   "n=2 min=1.98 max=2.00 mean=1.99 bad=1" "$(report "$CSV")"

# The floor check must not invent a spawn-z error out of a coerced 0.
floor() {
  awk -F, -v NUM="$HZ_NUMERIC_RE" \
      '$2!="NA" && $2!="" && $2 ~ NUM && $2+0 < 0.30 {n++} END{print n+0}' "$1"
}
is "poison is NOT counted as a below-0.30 m sample" "0" "$(floor "$CSV")"

printf '%s\n1754500050,0.03\n' "$(cat "$CSV")" > "$TMP/alt2.csv"
is "a genuine 0.03 m sample IS still counted" "1" "$(floor "$TMP/alt2.csv")"

echo ""
echo "passed $pass, failed $fail"
[ "$fail" -eq 0 ]
