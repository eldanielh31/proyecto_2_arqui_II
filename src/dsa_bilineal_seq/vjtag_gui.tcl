# =============================================================================
# vjtag_gui.tcl — GUI para Virtual JTAG (IR=2b: WRITE=1, READ=2)
# Acciones:
#   - Connect / Close
#   - Set Params (IN_W, IN_H, SCALE_Q88)
#   - Start
#   - Read Status / Read IN_W
#   - Dump mem_in / Dump mem_out
#   - Upload Input HEX…  (sube archivo .hex a BRAM de entrada)
#   - Read Perf (FLOPs / mem_rd / mem_wr)
# =============================================================================

if {[catch {package require Tk} err]} {
  puts "ERROR: No se pudo cargar Tk: $err"
  puts "Sugerencia: ejecute con el ejecutable de Quartus (bin64\\quartus_stp.exe)."
  exit 1
}
package require Tcl 8.5

# --- Estado global ---
set IRW  2
set DRW  40
set IR_WRITE 1
set IR_READ  2

set INST -1
set __connected 0
set __hw ""
set __dev ""

# --- Direcciones de registros (coinciden con RTL) ---
set ADDR_CONTROL     0x00 ;# bit0: start
set ADDR_IN_W        0x01
set ADDR_IN_H        0x02
set ADDR_SCALE_Q88   0x03
set ADDR_STATUS      0x10 ;# bit0: done, bit1: busy, bit2: error

# Performance counters / progreso
set ADDR_PERF_FLOPS  0x11
set ADDR_PERF_MEM_RD 0x12
set ADDR_PERF_MEM_WR 0x13
set ADDR_PROGRESS    0x14

# BRAM ENTRADA (view)
set ADDR_IN_ADDR     0x20
set ADDR_IN_DATA     0x21
# BRAM SALIDA (view)
set ADDR_OUT_ADDR    0x30
set ADDR_OUT_DATA    0x31
# BRAM ENTRADA (upload)
set ADDR_IN_WADDR    0x22
set ADDR_IN_WDATA    0x23

# =============================================================================
# Helpers
# =============================================================================
proc parse_uint {s} {
  set s [string trim $s]
  if {[string match -nocase 0x* $s]} {
    if {[scan $s %x v] != 1} { error "Valor inválido: $s" }
  } else {
    if {[scan $s %d v] != 1} { error "Valor inválido: $s" }
  }
  return [expr {$v & 0xFFFFFFFF}]
}

proc dr_parse_hex {s} {
  set hexWidth [expr {$::DRW / 4}]
  set s_trim   [string toupper [string trim $s]]
  if {[string length $s_trim] != $hexWidth} {
    error "DR inesperado: '$s_trim' (esperado $hexWidth dígitos hex)"
  }
  if {![string is xdigit -strict $s_trim]} { error "Respuesta no es HEX válido: '$s_trim'" }
  if {[scan $s_trim %x v] != 1} { error "No se pudo parsear HEX: '$s_trim'" }
  return $v
}

proc pack_dr {addr data32} {
  set a [expr {$addr & 0xFF}]
  set d [expr {$data32 & 0xFFFFFFFF}]
  return [expr {($d << 8) | $a}]
}

proc require_inst {} {
  if {!$::__connected} { error "No hay dispositivo abierto. Conecte primero." }
  if {$::INST < 0}     { error "No hay instancia Virtual JTAG. Presione Connect." }
}

# =============================================================================
# vJTAG primitives
# =============================================================================
proc vjtag_ir {val} {
  require_inst
  device_lock -timeout 10000
  set r [device_virtual_ir_shift -instance_index $::INST -ir_value $val]
  device_unlock
  return $r
}
proc vjtag_dr {val40} {
  require_inst
  set hexWidth [expr {$::DRW / 4}]
  set hex [format %0*X $hexWidth $val40]
  device_lock -timeout 10000
  set out [device_virtual_dr_shift -instance_index $::INST -dr_value $hex -length $::DRW -value_in_hex]
  device_unlock
  return $out
}

# =============================================================================
# Read/Write registros
# =============================================================================
proc write_reg {addr data} {
  vjtag_ir $::IR_WRITE
  set dr [pack_dr $addr $data]
  vjtag_dr $dr
}
proc read_reg {addr} {
  vjtag_ir $::IR_READ
  set dr1 [pack_dr $addr 0]
  vjtag_dr $dr1
  set rsp [vjtag_dr 0]
  set v   [dr_parse_hex $rsp]
  return [expr {($v >> 8) & 0xFFFFFFFF}]
}

# =============================================================================
# BRAM ENTRADA/SALIDA: leer (latencia 1 ciclo), escribir (auto-inc)
# =============================================================================
proc write_in_addr {addr}  { write_reg $::ADDR_IN_ADDR  $addr }
proc write_out_addr {addr} { write_reg $::ADDR_OUT_ADDR $addr }

proc read_in_data {} {
  vjtag_ir $::IR_READ
  set dr1 [pack_dr $::ADDR_IN_DATA 0]
  vjtag_dr $dr1
  set _   [vjtag_dr 0]
  set dr2 [pack_dr $::ADDR_IN_DATA 0]
  vjtag_dr $dr2
  set rsp [vjtag_dr 0]
  set v   [dr_parse_hex $rsp]
  return [expr {($v >> 8) & 0xFF}]
}
proc read_out_data {} {
  vjtag_ir $::IR_READ
  set dr1 [pack_dr $::ADDR_OUT_DATA 0]
  vjtag_dr $dr1
  set _   [vjtag_dr 0]
  set dr2 [pack_dr $::ADDR_OUT_DATA 0]
  vjtag_dr $dr2
  set rsp [vjtag_dr 0]
  set v   [dr_parse_hex $rsp]
  return [expr {($v >> 8) & 0xFF}]
}

# Escritura de bytes a mem_in con autoincremento en HW
proc write_in_waddr {addr} { write_reg $::ADDR_IN_WADDR $addr }
proc write_in_wdata {byte} { write_reg $::ADDR_IN_WDATA $byte }

# =============================================================================
# Gestión de hardware / dispositivo
# =============================================================================
proc refresh_hwlist {} {
  .f.hwList delete 0 end
  foreach h [get_hardware_names] { .f.hwList insert end $h }
}
proc refresh_devlist {} {
  .f.devList delete 0 end
  set sel [.f.hwList curselection]
  if {$sel eq ""} { return }
  set hw [.f.hwList get [lindex $sel 0]]
  foreach d [get_device_names -hardware_name $hw] { .f.devList insert end $d }
}
proc auto_select_instance {} {
  if {!$::__connected} { error "Abra conexión primero" }
  for {set i 0} {$i < 32} {incr i} {
    set ok [catch {
      device_lock -timeout 3000
      set _ [device_virtual_ir_shift -instance_index $i -ir_value 0]
      device_unlock
    } err]
    if {$ok == 0} { set ::INST $i; return $i }
  }
  set ::INST -1
  return -1
}
proc jtag_open {} {
  set selh [.f.hwList curselection]
  if {$selh eq ""} { tk_messageBox -icon error -message "Seleccione un hardware."; return 0 }
  set seld [.f.devList curselection]
  if {$seld eq ""} { tk_messageBox -icon error -message "Seleccione un device."; return 0 }
  set hw  [.f.hwList get [lindex $selh 0]]
  set dev [.f.devList get [lindex $seld 0]]
  open_device -hardware_name $hw -device_name $dev
  set ::__connected 1
  set ::__hw $hw
  set ::__dev $dev
  set idx [auto_select_instance]
  if {$idx < 0} {
    tk_messageBox -icon error -message "Conectado a:\n$hw / $dev\nPero no se encontró Virtual JTAG.\n¿Reprogramó el .sof con el IP vJTAG?"
  } else {
    tk_messageBox -message "Conectado a:\n$hw / $dev\nVirtual JTAG instance_index = $idx"
  }
  return 1
}
proc jtag_close {} {
  if {$::__connected} { catch { close_device } }
  set ::__connected 0
  set ::INST -1
  set ::__hw ""
  set ::__dev ""
}

# =============================================================================
# Utilidades GUI
# =============================================================================
proc compute_out_dims {} {
  set W [read_reg $::ADDR_IN_W]
  set H [read_reg $::ADDR_IN_H]
  set S [read_reg $::ADDR_SCALE_Q88]
  set Wp [expr {int( ($W * $S) / 256 )}]
  set Hp [expr {int( ($H * $S) / 256 )}]
  return [list $Wp $Hp]
}

proc dump_mem_in {start count filepath} {
  require_inst
  set fd [open $filepath "w"]
  fconfigure $fd -translation lf
  for {set i 0} {$i < $count} {incr i} {
    set addr [expr {$start + $i}]
    write_in_addr $addr
    set b [read_in_data]
    puts $fd [format "%02x" $b]
  }
  close $fd
  tk_messageBox -message "Dump IN OK: $count bytes desde $start a '$filepath'"
}
proc dump_mem_out {start count filepath} {
  require_inst
  set fd [open $filepath "w"]
  fconfigure $fd -translation lf
  for {set i 0} {$i < $count} {incr i} {
    set addr [expr {$start + $i}]
    write_out_addr $addr
    set b [read_out_data]
    puts $fd [format "%02x" $b]
  }
  close $fd
  tk_messageBox -message "Dump OUT OK: $count bytes desde $start a '$filepath'"
}

# Subir un .hex a BRAM de entrada (una línea = 2 dígitos hex por byte)
proc upload_hex_to_input {{base_addr 0}} {
  require_inst
  set path [tk_getOpenFile -title "Seleccione archivo .hex" -filetypes {{"HEX Files" {.hex}} {"All Files" {*}}}]
  if {$path eq ""} { return }
  set fd [open $path "r"]
  fconfigure $fd -translation lf
  write_in_waddr $base_addr
  set n 0
  while {[gets $fd line] >= 0} {
    set line [string trim $line]
    if {$line eq ""} { continue }
    if {![string is xdigit -strict $line]} { continue }
    if {[string length $line] > 2} { set line [string range $line end-1 end] }
    if {[scan $line %x v] != 1} { continue }
    write_in_wdata [expr {$v & 0xFF}]
    incr n
  }
  close $fd
  tk_messageBox -message "Upload OK: $n bytes escritos a mem_in desde addr $base_addr"
}

# =============================================================================
# Construcción GUI
# =============================================================================
wm title . "Virtual JTAG GUI (DE1-SoC)"

frame .f
grid .f -padx 8 -pady 8 -sticky news
grid columnconfigure .f 0 -weight 1
grid columnconfigure .f 1 -weight 1

label  .f.lhw -text "Hardware"
listbox .f.hwList -width 50 -height 4 -exportselection 0
button .f.btnRhw -text "Refresh HW" -command { refresh_hwlist; .f.devList delete 0 end }

label  .f.ldv -text "Device"
listbox .f.devList -width 50 -height 4 -exportselection 0
button .f.btnRdv -text "Refresh Dev" -command { refresh_devlist }

grid .f.lhw     -row 0 -column 0 -sticky w
grid .f.btnRhw  -row 0 -column 1 -sticky e
grid .f.hwList  -row 1 -column 0 -columnspan 2 -sticky we -padx 4 -pady 2

grid .f.ldv     -row 2 -column 0 -sticky w -pady 4
grid .f.btnRdv  -row 2 -column 1 -sticky e
grid .f.devList -row 3 -column 0 -columnspan 2 -sticky we -padx 4 -pady 2

button .f.btnConnect -text "Connect" -command { jtag_open }
button .f.btnClose   -text "Close"   -command { jtag_close }
grid   .f.btnConnect -row 4 -column 0 -padx 4 -pady 6 -sticky w
grid   .f.btnClose   -row 4 -column 1 -padx 4 -pady 6 -sticky e

label .f.lw -text "in_w"
entry .f.ew -width 12
label .f.lh -text "in_h"
entry .f.eh -width 12
label .f.ls -text "scale_q88 (Q8.8)"
entry .f.es -width 12

grid .f.lw -row 5 -column 0 -sticky e
grid .f.ew -row 5 -column 1 -sticky w
grid .f.lh -row 6 -column 0 -sticky e
grid .f.eh -row 6 -column 1 -sticky w
grid .f.ls -row 7 -column 0 -sticky e
grid .f.es -row 7 -column 1 -sticky w

button .f.btnSet   -text "Set Params" -command {
  if {$::INST < 0} { tk_messageBox -icon error -message "No Virtual JTAG instance. Presione Connect primero."; return }
  set w [parse_uint [.f.ew get]]
  set h [parse_uint [.f.eh get]]
  set s [parse_uint [.f.es get]]
  write_reg $::ADDR_IN_W      $w
  write_reg $::ADDR_IN_H      $h
  write_reg $::ADDR_SCALE_Q88 $s
  tk_messageBox -message "Params written"
}
button .f.btnStart -text "Start" -command {
  if {$::INST < 0} { tk_messageBox -icon error -message "No Virtual JTAG instance. Presione Connect primero."; return }
  write_reg $::ADDR_CONTROL 1
  catch {
    vjtag_ir $::IR_READ
    set _dr1 [pack_dr $::ADDR_STATUS 0]
    vjtag_dr $_dr1
    vjtag_dr 0
  }
}
grid .f.btnSet   -row 8 -column 0 -pady 6 -sticky w
grid .f.btnStart -row 8 -column 1 -pady 6 -sticky e

label .f.lst -text "status"
entry .f.est -width 16
button .f.btnRd -text "Read Status" -command {
  if {$::INST < 0} { tk_messageBox -icon error -message "No Virtual JTAG instance. Presione Connect primero."; return }
  set d [read_reg $::ADDR_STATUS]
  .f.est delete 0 end
  .f.est insert 0 $d
}
grid .f.lst  -row 9 -column 0 -sticky e
grid .f.est  -row 9 -column 1 -sticky w
grid .f.btnRd -row 10 -column 0 -pady 6 -sticky w

button .f.btnRdW -text "Read IN_W" -command {
  if {$::INST < 0} { tk_messageBox -icon error -message "No Virtual JTAG instance. Presione Connect primero."; return }
  set w [read_reg $::ADDR_IN_W]
  tk_messageBox -message "IN_W = $w"
}
grid .f.btnRdW -row 10 -column 1 -pady 6 -sticky e

# ---- Dump Input BRAM ----
label .f.i_hdr -text "Dump Input BRAM (mem_in)"
grid  .f.i_hdr -row 11 -column 0 -columnspan 2 -sticky w -pady 6

label .f.i_start -text "start (dec/hex)"
entry .f.i_estart -width 12
label .f.i_count -text "count"
entry .f.i_ecount -width 12
label .f.i_file  -text "file"
entry .f.i_efile -width 24

grid .f.i_start -row 12 -column 0 -sticky e
grid .f.i_estart -row 12 -column 1 -sticky w
grid .f.i_count -row 13 -column 0 -sticky e
grid .f.i_ecount -row 13 -column 1 -sticky w
grid .f.i_file  -row 14 -column 0 -sticky e
grid .f.i_efile -row 14 -column 1 -sticky w

button .f.i_dump -text "Dump Input" -command {
  if {$::INST < 0} { tk_messageBox -icon error -message "No Virtual JTAG instance. Presione Connect primero."; return }
  set start [parse_uint [.f.i_estart get]]
  set count [parse_uint [.f.i_ecount get]]
  set file  [.f.i_efile get]
  if {$file eq ""} { set file "mem_in_dump.hex" }
  dump_mem_in $start $count $file
}
grid .f.i_dump -row 15 -column 1 -pady 6 -sticky e

# ---- Upload Input HEX (nuevo) ----
button .f.i_upload -text "Upload Input HEX…" -command {
  if {$::INST < 0} { tk_messageBox -icon error -message "No Virtual JTAG instance. Presione Connect primero."; return }
  set base [parse_uint [.f.i_estart get]]
  upload_hex_to_input $base
}
grid .f.i_upload -row 15 -column 0 -pady 6 -sticky w

# ---- Dump Output BRAM ----
label .f.o_hdr -text "Dump Output BRAM (mem_out)"
grid  .f.o_hdr -row 16 -column 0 -columnspan 2 -sticky w -pady 6

label .f.o_start -text "start (dec/hex)"
entry .f.o_estart -width 12
label .f.o_count -text "count"
entry .f.o_ecount -width 12
label .f.o_file  -text "file"
entry .f.o_efile -width 24

grid .f.o_start -row 17 -column 0 -sticky e
grid .f.o_estart -row 17 -column 1 -sticky w
grid .f.o_count -row 18 -column 0 -sticky e
grid .f.o_ecount -row 18 -column 1 -sticky w
grid .f.o_file  -row 19 -column 0 -sticky e
grid .f.o_efile -row 19 -column 1 -sticky w

button .f.o_autofill -text "Auto-fill W'*H'" -command {
  if {$::INST < 0} { tk_messageBox -icon error -message "No Virtual JTAG instance. Presione Connect primero."; return }
  lassign [compute_out_dims] Wp Hp
  .f.o_estart delete 0 end
  .f.o_estart insert 0 0
  .f.o_ecount delete 0 end
  .f.o_ecount insert 0 [expr {$Wp * $Hp}]
}
button .f.o_dump -text "Dump Output" -command {
  if {$::INST < 0} { tk_messageBox -icon error -message "No Virtual JTAG instance. Presione Connect primero."; return }
  set start [parse_uint [.f.o_estart get]]
  set count [parse_uint [.f.o_ecount get]]
  set file  [.f.o_efile get]
  if {$file eq ""} { set file "mem_out.hex" }
  dump_mem_out $start $count $file
}
grid .f.o_autofill -row 20 -column 0 -pady 6 -sticky w
grid .f.o_dump     -row 20 -column 1 -pady 6 -sticky e

# ---- Performance counters ----
label .f.lperf -text "Perf (FLOPs / mem_rd / mem_wr)"
entry .f.eperf -width 40
button .f.btnPerf -text "Read Perf" -command {
  if {$::INST < 0} { tk_messageBox -icon error -message "No Virtual JTAG instance. Presione Connect primero."; return }
  set fl [read_reg $::ADDR_PERF_FLOPS]
  set rd [read_reg $::ADDR_PERF_MEM_RD]
  set wr [read_reg $::ADDR_PERF_MEM_WR]
  .f.eperf delete 0 end
  .f.eperf insert 0 "$fl / $rd / $wr"
}
grid .f.lperf  -row 22 -column 0 -sticky e
grid .f.eperf  -row 22 -column 1 -sticky w
grid .f.btnPerf -row 23 -column 1 -pady 4 -sticky e

button .f.btnQuit -text "Quit" -command {
  jtag_close
  set ::__exit 1
}
grid   .f.btnQuit -row 24 -column 1 -pady 6 -sticky e

# Defaults
.f.ew insert 0 64
.f.eh insert 0 64
.f.es insert 0 205

.f.i_estart insert 0 0
.f.i_ecount insert 0 4096
.f.i_efile  insert 0 "mem_in_dump.hex"

.f.o_estart insert 0 0
.f.o_ecount insert 0 4096
.f.o_efile  insert 0 "mem_out.hex"

after 0 refresh_hwlist
bind .f.hwList <<ListboxSelect>> { refresh_devlist }

wm protocol . WM_DELETE_WINDOW { jtag_close ; set ::__exit 1 }
vwait ::__exit
