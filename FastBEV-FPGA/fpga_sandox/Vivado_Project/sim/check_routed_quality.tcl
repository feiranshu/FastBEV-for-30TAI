open_checkpoint out/04_route.dcp
report_methodology -file out/04_methodology_route.rpt
report_cdc -details -file out/04_cdc_route.rpt
set sa_cells [get_cells -hierarchical -filter {NAME =~ *U_bev_accel_top/U_sa_engine/*}]
report_timing -from $sa_cells -to $sa_cells -max_paths 20 -nworst 5 \
    -file out/04_timing_sa_only.rpt

set setup_path [get_timing_paths -delay_type max -max_paths 1 -nworst 1]
set hold_path  [get_timing_paths -delay_type min -max_paths 1 -nworst 1]
set sa_path    [get_timing_paths -from $sa_cells -to $sa_cells \
                    -delay_type max -max_paths 1 -nworst 1]
set setup_slack [get_property SLACK $setup_path]
set hold_slack  [get_property SLACK $hold_path]
set sa_slack    [get_property SLACK $sa_path]
puts "ROUTED_SLACK_NS setup=$setup_slack hold=$hold_slack sa=$sa_slack"
if {$setup_slack < 0.0 || $hold_slack < 0.0 || $sa_slack < 0.0} {
    error "Unprotected routed timing violation: setup=$setup_slack hold=$hold_slack sa=$sa_slack"
}

set timing_fh [open out/04_timing_route.rpt r]
set timing_text [read $timing_fh]
close $timing_fh
set protected_pw 0
set unprotected_pw {}
foreach timing_line [split $timing_text "\n"] {
    if {[regexp {^(Min Period|Max Period|Low Pulse Width|High Pulse Width).*\s-[0-9]+\.[0-9]+\s} $timing_line]} {
        if {[string first "U0_ps_ai_wrap_demo/" $timing_line] >= 0} {
            incr protected_pw
        } else {
            lappend unprotected_pw $timing_line
        }
    }
}
if {[llength $unprotected_pw] != 0} {
    error "Pulse-width violation outside protected EDIF: $unprotected_pw"
}
puts "PROTECTED_EDIF_PULSE_WIDTH_ROWS_IGNORED=$protected_pw"
puts "ROUTED_QUALITY_CHECK_PASS"
