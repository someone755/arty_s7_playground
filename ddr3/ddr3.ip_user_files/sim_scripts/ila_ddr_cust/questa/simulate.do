onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib ila_ddr_cust_opt

do {wave.do}

view wave
view structure
view signals

do {ila_ddr_cust.udo}

run -all

quit -force
