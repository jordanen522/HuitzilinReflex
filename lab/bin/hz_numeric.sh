# hz_numeric.sh — one definition of "is this token a number", sourceable.
#
# Why this exists. The altitude sampler ran
#
#     ros2 topic echo /huitzilin/odom --once --field pose.pose.position.z \
#       2>/dev/null | head -1 | tr -d " \r"
#
# and `ros2 topic echo` prints the DDS diagnostic "A message was lost!!!" on
# STDOUT, not stderr, so 2>/dev/null never saw it, head -1 took it, and
# `tr -d " "` compacted it to the token Amessagewaslost!!!. The reporting awk
# then admitted it (it only rejected "NA" and ""), coerced it to numeric 0,
# and the cell reported a 0.00 m altitude sample. No aircraft ever went there:
# every real sample in all five clean cells is >= 1.98 m. A measurement
# artifact was very nearly reported as vehicle behaviour.
#
# The regex lives HERE and nowhere else, so the sampler, the reporter and the
# test cannot drift apart. Accepts optional sign, decimal point and exponent;
# rejects everything else including the empty string.
HZ_NUMERIC_RE="^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$"

# first_numeric — echo the first line of stdin that is a number, else nothing.
# Deliberately a FILTER, not a "take line 1 and hope": any number of diagnostic
# lines may precede the value, and skipping them is the whole point.
first_numeric() {
  tr -d " \r" | grep -m1 -E "$HZ_NUMERIC_RE" || true
}
