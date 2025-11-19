# Clock principal de 50 MHz en la DE1-SoC
create_clock -name {clk_50} -period 20.000 [get_ports {clk_50}]
