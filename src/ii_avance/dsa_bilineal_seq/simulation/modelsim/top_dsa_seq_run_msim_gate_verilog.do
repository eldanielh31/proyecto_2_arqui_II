transcript on
if {[file exists gate_work]} {
	vdel -lib gate_work -all
}
vlib gate_work
vmap work gate_work

vlog -vlog01compat -work work +incdir+. {top_dsa_seq.vo}

vlog -sv -work work +incdir+C:/Users/danbg/src/proyecto_2_arqui_II/src/ii_avance/dsa_bilineal_seq {C:/Users/danbg/src/proyecto_2_arqui_II/src/ii_avance/dsa_bilineal_seq/tb_bilinear_seq.sv}

vsim -t 1ps +transport_int_delays +transport_path_delays -L altera_ver -L altera_lnsim_ver -L cyclonev_ver -L lpm_ver -L sgate_ver -L cyclonev_hssi_ver -L altera_mf_ver -L cyclonev_pcie_hip_ver -L gate_work -L work -voptargs="+acc"  tb_bilinear_seq

add wave *
view structure
view signals
run -all
