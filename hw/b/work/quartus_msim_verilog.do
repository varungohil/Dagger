catch {
file mkdir ./verilog_libs/altera_ver
vlib "./verilog_libs/altera_ver"
vmap altera_ver "./verilog_libs/altera_ver"
vlog -work altera_ver -vlog01compat "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/altera_primitives.v" 
file mkdir ./verilog_libs/lpm_ver
vlib "./verilog_libs/lpm_ver"
vmap lpm_ver "./verilog_libs/lpm_ver"
vlog -work lpm_ver -vlog01compat "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/220model.v" 
file mkdir ./verilog_libs/sgate_ver
vlib "./verilog_libs/sgate_ver"
vmap sgate_ver "./verilog_libs/sgate_ver"
vlog -work sgate_ver -vlog01compat "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/sgate.v" 
file mkdir ./verilog_libs/altera_mf_ver
vlib "./verilog_libs/altera_mf_ver"
vmap altera_mf_ver "./verilog_libs/altera_mf_ver"
vlog -work altera_mf_ver -vlog01compat "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/altera_mf.v" 
file mkdir ./verilog_libs/altera_lnsim_ver
vlib "./verilog_libs/altera_lnsim_ver"
vmap altera_lnsim_ver "./verilog_libs/altera_lnsim_ver"
vlog -work altera_lnsim_ver -sv "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/altera_lnsim.sv" 
file mkdir ./verilog_libs/twentynm_ver
vlib "./verilog_libs/twentynm_ver"
vmap twentynm_ver "./verilog_libs/twentynm_ver"
vlog -work twentynm_ver -vlog01compat "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/twentynm_atoms.v" 
vlog -work twentynm_ver -vlog01compat "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/mentor/twentynm_atoms_ncrypt.v" 
file mkdir ./verilog_libs/twentynm_hssi_ver
vlib "./verilog_libs/twentynm_hssi_ver"
vmap twentynm_hssi_ver "./verilog_libs/twentynm_hssi_ver"
vlog -work twentynm_hssi_ver -vlog01compat "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/mentor/twentynm_hssi_atoms_ncrypt.v" 
vlog -work twentynm_hssi_ver -vlog01compat "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/twentynm_hssi_atoms.v" 
file mkdir ./verilog_libs/twentynm_hip_ver
vlib "./verilog_libs/twentynm_hip_ver"
vmap twentynm_hip_ver "./verilog_libs/twentynm_hip_ver"
vlog -work twentynm_hip_ver -vlog01compat "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/mentor/twentynm_hip_atoms_ncrypt.v" 
vlog -work twentynm_hip_ver -vlog01compat "/export/fpga/tools/quartus_pro/17.1.1/quartus/eda/sim_lib/twentynm_hip_atoms.v" 
quit -f
} result
puts "Library compilation terminated due to error in compilation( $result )"
quit -f
