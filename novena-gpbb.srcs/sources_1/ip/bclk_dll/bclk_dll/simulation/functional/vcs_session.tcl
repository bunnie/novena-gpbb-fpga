gui_open_window Wave
gui_sg_create bclk_dll_group
gui_list_add_group -id Wave.1 {bclk_dll_group}
gui_sg_addsignal -group bclk_dll_group {bclk_dll_tb.test_phase}
gui_set_radix -radix {ascii} -signals {bclk_dll_tb.test_phase}
gui_sg_addsignal -group bclk_dll_group {{Input_clocks}} -divider
gui_sg_addsignal -group bclk_dll_group {bclk_dll_tb.CLK_IN1}
gui_sg_addsignal -group bclk_dll_group {{Output_clocks}} -divider
gui_sg_addsignal -group bclk_dll_group {bclk_dll_tb.dut.clk}
gui_list_expand -id Wave.1 bclk_dll_tb.dut.clk
gui_sg_addsignal -group bclk_dll_group {{Status_control}} -divider
gui_sg_addsignal -group bclk_dll_group {bclk_dll_tb.RESET}
gui_sg_addsignal -group bclk_dll_group {bclk_dll_tb.LOCKED}
gui_sg_addsignal -group bclk_dll_group {{Counters}} -divider
gui_sg_addsignal -group bclk_dll_group {bclk_dll_tb.COUNT}
gui_sg_addsignal -group bclk_dll_group {bclk_dll_tb.dut.counter}
gui_list_expand -id Wave.1 bclk_dll_tb.dut.counter
gui_zoom -window Wave.1 -full
