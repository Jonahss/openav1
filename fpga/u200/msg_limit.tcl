# Run hook (fpga/u200/build.tcl): no cut-off of repeated messages, so tools/warn_census.py sees every warning.
set_param messaging.defaultLimit 1000000
