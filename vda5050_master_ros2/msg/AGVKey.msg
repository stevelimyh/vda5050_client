# AGVKey — minimal identity pair for any AGV-addressed API that doesn't
# need queue config or other onboarding parameters. Used by
# OffboardAGV.srv (single) and OffboardAGVBatch.srv (batch), and can be
# reused by any future fleet-wide AGV-targeting service.

string manufacturer
string serial_number
