onbreak {quit -force}
onerror {quit -force}

asim -t 1ps +access +r +m+ddr3l -L xpm -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip -O5 xil_defaultlib.ddr3l xil_defaultlib.glbl

do {wave.do}

view wave
view structure

do {ddr3l.udo}

run -all

endsim

quit -force
