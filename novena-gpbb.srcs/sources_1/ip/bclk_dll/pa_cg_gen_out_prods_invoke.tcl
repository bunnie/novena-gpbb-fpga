# Tcl script generated by PlanAhead

set reloadAllCoreGenRepositories false

set tclUtilsPath "c:/Xilinx/14.5/ISE_DS/PlanAhead/scripts/pa_cg_utils.tcl"

set repoPaths ""

set cgIndexMapPath "C:/local/local_copy/novena-gpbb/novena-gpbb.srcs/sources_1/ip/cg_nt_index_map.xml"

set cgProjectPath "c:/local/local_copy/novena-gpbb/novena-gpbb.srcs/sources_1/ip/bclk_dll/coregen.cgc"

set ipFile "c:/local/local_copy/novena-gpbb/novena-gpbb.srcs/sources_1/ip/bclk_dll/bclk_dll.xco"

set ipName "bclk_dll"

set hdlType "Verilog"

set cgPartSpec "xc6slx45-3csg324"

set chains "GENERATE_CURRENT_CHAIN"

set params ""

set bomFilePath "c:/local/local_copy/novena-gpbb/novena-gpbb.srcs/sources_1/ip/bclk_dll/pa_cg_bom.xml"

# generate the IP
set result [source "c:/Xilinx/14.5/ISE_DS/PlanAhead/scripts/pa_cg_gen_out_prods.tcl"]

exit $result

