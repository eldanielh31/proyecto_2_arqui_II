transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vlog -sv -work work +incdir+C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq {C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/wide_onchip_mem.sv}
vlog -sv -work work +incdir+C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq {C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/bilinear_simd4_wide.sv}
vlog -sv -work work +incdir+C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq {C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/bilinear_seq_wide.sv}
vlog -sv -work work +incdir+C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq {C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/top_dsa_seq.sv}

vlog -sv -work work +incdir+C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq {C:/Users/danbg/src/proyecto_2_arqui_II/src/dsa_bilineal_seq/tb_bilinear_seq.sv}

vsim -t 1ps -L altera_ver -L lpm_ver -L sgate_ver -L altera_mf_ver -L altera_lnsim_ver -L cyclonev_ver -L cyclonev_hssi_ver -L cyclonev_pcie_hip_ver -L rtl_work -L work -voptargs="+acc"  tb_bilinear_seq

add wave *
view structure
view signals
run -all
