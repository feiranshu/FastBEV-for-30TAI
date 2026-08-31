# Full standalone synth/implementation check from an ASCII-only staging path.
set output_dir [file normalize "out"]
file mkdir $output_dir
create_project -in_memory -part xc7z030ffg676-2
set_property target_language Verilog [current_project]
read_verilog [glob rtl/*.v]
read_edif rtl/ps_ai_wrap_demo.edf
read_xdc constrs/impl_constraints.xdc
read_xdc constrs/ai7030.xdc

synth_design -top bev_edif_top -part xc7z030ffg676-2
report_utilization -file "$output_dir/01_utilization_synth.rpt"
write_checkpoint -force "$output_dir/01_synth.dcp"

opt_design
report_utilization -file "$output_dir/02_utilization_opt.rpt"
report_drc -file "$output_dir/02_drc_opt.rpt"
write_checkpoint -force "$output_dir/02_opt.dcp"

place_design
report_utilization -file "$output_dir/03_utilization_place.rpt"
report_timing_summary -file "$output_dir/03_timing_place.rpt"
write_checkpoint -force "$output_dir/03_place.dcp"

phys_opt_design
route_design
report_route_status -file "$output_dir/04_route_status.rpt"
report_timing_summary -file "$output_dir/04_timing_route.rpt"
report_drc -file "$output_dir/04_drc_route.rpt"
report_methodology -file "$output_dir/04_methodology_route.rpt"
report_cdc -details -file "$output_dir/04_cdc_route.rpt"
set sa_cells [get_cells -hierarchical -filter {NAME =~ *U_bev_accel_top/U_sa_engine/*}]
report_timing -from $sa_cells -to $sa_cells -max_paths 20 -nworst 5 \
    -file "$output_dir/04_timing_sa_only.rpt"
write_checkpoint -force "$output_dir/04_route.dcp"

# Treat setup/hold as hard failures. Pulse-width failures are ignored only when
# every negative-slack row belongs to the immutable ps_ai_wrap_demo EDIF.
set setup_path [get_timing_paths -delay_type max -max_paths 1 -nworst 1]
set hold_path  [get_timing_paths -delay_type min -max_paths 1 -nworst 1]
set sa_path    [get_timing_paths -from $sa_cells -to $sa_cells \
                    -delay_type max -max_paths 1 -nworst 1]
if {[llength $setup_path] == 0 || [llength $hold_path] == 0 || \
    [llength $sa_path] == 0} {
    error "Unable to obtain routed setup/hold/SA timing paths"
}
set setup_slack [get_property SLACK $setup_path]
set hold_slack  [get_property SLACK $hold_path]
set sa_slack    [get_property SLACK $sa_path]
puts "ROUTED_SLACK_NS setup=$setup_slack hold=$hold_slack sa=$sa_slack"
if {$setup_slack < 0.0 || $hold_slack < 0.0 || $sa_slack < 0.0} {
    error "Unprotected routed timing violation: setup=$setup_slack hold=$hold_slack sa=$sa_slack"
}

set timing_fh [open "$output_dir/04_timing_route.rpt" r]
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
puts "FULL_ASCII_IMPLEMENT_CHECK_PASS"
