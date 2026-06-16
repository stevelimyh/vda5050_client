# AgvAlignment — one onboarded AGV's alignment status against the
# master's loaded map. Returned inside GetLoadedMap.srv responses.
#
# An AGV with has_factsheet=false has not yet sent a factsheet to the
# master, so no alignment can be computed yet — findings is empty and
# has_error is false. The first factsheet received (or first map load
# after the factsheet was already cached) triggers a fresh check.

string manufacturer
string serial_number

# True if the master has at least one cached factsheet for this AGV at
# the time the alignment was computed. When false, no alignment was
# performed; findings is empty.
bool has_factsheet

# True if at least one finding has severity ERROR. Convenience flag for
# operators scanning a fleet at a glance — equivalent to scanning
# findings for any SEVERITY_ERROR.
bool has_error

# Total number of findings (warnings + errors). Useful when the
# findings array is truncated by a future limit; in V0 the array
# carries every finding so finding_count == findings.size().
uint32 finding_count

# All findings for this AGV. Empty if the alignment is fully clean or
# if has_factsheet=false.
AgvAlignmentFinding[] findings
