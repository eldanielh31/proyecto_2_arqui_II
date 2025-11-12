# vjtag_gui.tcl — GUI simple para Virtual JTAG (IR=2: WRITE=1, READ=2)
# Ejecutar:
#   "C:\intelFPGA_lite\18.1\quartus\bin64\quartus_stp.exe" -t vjtag_gui.tcl

# --- Cargar Tk y validar ---
if {[catch {package require Tk} err]} {
  puts "ERROR: No se pudo cargar Tk: $err"
  puts "Sugerencia: ejecute con el ejecutable de Quartus (bin64\\quartus_stp.exe)."
  exit 1
}
package require Tcl 8.5

# === Estado global ===
set IRW 2            ;# informativo
set DRW 40           ;# 8 addr + 32 data
set INST -1
set __connected 0
set __hw ""
set __dev ""

# === Helpers de parseo ===
proc parse_uint {s} {
  if {[string match 0x* $s] || [string match 0X* $s]} {
    if {[scan $s %x v] != 1} { error "Valor inválido: $s" }
  } else {
    if {[scan $s %d v] != 1} { error "Valor inválido: $s" }
  }
  return [expr {$v & 0xFFFFFFFF}]
}

# Convierte la salida de device_virtual_dr_shift a entero:
#  - Soporta "captured_dr_value N" (decimal)
#  - Soporta "XXXXXXXXXX" (HEX puro de DRW/4 dígitos)
proc parse_captured_value {s} {
  # 1) Formato con etiqueta
  if {[regexp {captured_dr_value\s+(\d+)} $s -> dec]} {
    return $dec
  }
  # 2) HEX puro
  set hexWidth [expr {$::DRW / 4}]          ;# p.ej. 40 -> 10
  set s_trim   [string trim $s]
  if {[string length $s_trim] != $hexWidth} {
    error "Longitud inesperada del DR: '$s_trim' (esperado $hexWidth hex dígitos)"
  }
  if {![string is xdigit -strict $s_trim]} {
    error "Respuesta no es HEX válido de $hexWidth dígitos: '$s_trim'"
  }
  if {[scan $s_trim %x v] != 1} {
    error "No se pudo parsear HEX: '$s_trim'"
  }
  return $v
}

# === Shifts (STP 18.1) ===
proc vjtag_ir {val} {
  if {!$::__connected} { error "No device open" }
  if {$::INST < 0}     { error "Virtual JTAG instance not selected" }
  device_lock -timeout 10000
  set r [device_virtual_ir_shift -instance_index $::INST -ir_value $val]
  device_unlock
  return $r
}
proc vjtag_dr {val} {
  if {!$::__connected} { error "No device open" }
  if {$::INST < 0}     { error "Virtual JTAG instance not selected" }
  set hexWidth [expr {$::DRW / 4}]         ;# 40 -> 10 hex
  set hex [format %0*X $hexWidth $val]     ;# sin '0x'
  device_lock -timeout 10000
  set out [device_virtual_dr_shift -instance_index $::INST -dr_value $hex -length $::DRW -value_in_hex]
  device_unlock
  return $out  ;# puede ser "captured_dr_value N" o "XXXXXXXXXX"
}

# === Auto-descubrimiento del instance_index ===
proc auto_select_instance {} {
  if {!$::__connected} { error "Abra conexión primero" }
  for {set i 0} {$i < 32} {incr i} {
    set ok [catch {
      device_lock -timeout 3000
      set _ [device_virtual_ir_shift -instance_index $i -ir_value 0]
      device_unlock
    } err]
    if {$ok == 0} {
      set ::INST $i
      return $i
    }
  }
  set ::INST -1
  return -1
}

# Empaquetado del DR: [39:8]=data, [7:0]=addr
proc pack_dr {addr data32} {
  set a [expr {$addr & 0xFF}]
  set d [expr {$data32 & 0xFFFFFFFF}]
  return [expr {($d << 8) | $a}]
}

# === Direcciones de registros ===
set ADDR_CONTROL   0x00
set ADDR_IN_W      0x01
set ADDR_IN_H      0x02
set ADDR_SCALE_Q88 0x03
set ADDR_STATUS    0x10

# === Acceso de registros ===
proc write_reg {addr data} {
  vjtag_ir 1
  set dr [pack_dr $addr $data]
  vjtag_dr $dr
}
proc read_reg {addr} {
  vjtag_ir 2
  # 1) enviar addr
  set dr1 [pack_dr $addr 0]
  vjtag_dr $dr1
  # 2) ciclo extra para capturar
  set rsp [vjtag_dr 0]
  set val [parse_captured_value $rsp]
  # extraer data [39:8]
  set data [expr {($val >> 8) & 0xFFFFFFFF}]
  return $data
}

# === Gestión de conexión: selección de hardware y device ===
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
    tk_messageBox -icon error -message "Conectado a:\n$hw / $dev\nPero no se encontró Virtual JTAG.\n¿Reprogramó el .sof con el IP vJTAG y lo conectó a la lógica (no optimizado fuera)?"
  } else {
    tk_messageBox -message "Conectado a:\n$hw / $dev\nVirtual JTAG instance_index = $idx"
  }
  return 1
}
proc jtag_close {} {
  if {$::__connected} {
    catch { close_device }
  }
  set ::__connected 0
  set ::INST -1
  set ::__hw ""
  set ::__dev ""
}

# === GUI ===
wm title . "Virtual JTAG GUI (DE1-SoC)"

frame .f
grid .f -padx 8 -pady 8 -sticky news
grid columnconfigure .f 0 -weight 1
grid columnconfigure .f 1 -weight 1

# Selección HW/Device
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

# Entradas
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

# Botones
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
  # 1) Escribir CONTROL.bit0=1
  write_reg $::ADDR_CONTROL 1
  # 2) FLUSH TAP (oculto): un read rápido de STATUS garantiza que el escaneo se ejecute ya
  catch {
    vjtag_ir 2
    set _dr1 [pack_dr $::ADDR_STATUS 0]
    vjtag_dr $_dr1
    vjtag_dr 0
  }
}
grid .f.btnSet   -row 8 -column 0 -pady 6 -sticky w
grid .f.btnStart -row 8 -column 1 -pady 6 -sticky e

# Lecturas
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

# Prueba rápida de lectura (lee IN_W para validar DR de retorno)
button .f.btnRdW -text "Read IN_W" -command {
  if {$::INST < 0} { tk_messageBox -icon error -message "No Virtual JTAG instance. Presione Connect primero."; return }
  set w [read_reg $::ADDR_IN_W]
  tk_messageBox -message "IN_W = $w"
}
grid .f.btnRdW -row 10 -column 1 -pady 6 -sticky e

# Quit
button .f.btnQuit -text "Quit" -command {
  jtag_close
  set ::__exit 1
}
grid   .f.btnQuit -row 11 -column 1 -pady 6 -sticky e

# Defaults
.f.ew insert 0 64
.f.eh insert 0 64
.f.es insert 0 205

# Poblar listas al inicio
after 0 refresh_hwlist
bind .f.hwList <<ListboxSelect>> { refresh_devlist }

# Mantener GUI
wm protocol . WM_DELETE_WINDOW { jtag_close ; set ::__exit 1 }
vwait ::__exit
