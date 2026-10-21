// Black box for hierarchical linking to the verified packed OOC checkpoint.
(* black_box="true" *) module vitseg_core(ap_clk,ap_rst_n,image_in_TDATA,image_in_TVALID,image_in_TREADY,centers_out_TDATA,centers_out_TVALID,centers_out_TREADY,s_axi_ctrl_AWVALID,s_axi_ctrl_AWREADY,s_axi_ctrl_AWADDR,s_axi_ctrl_WVALID,s_axi_ctrl_WREADY,s_axi_ctrl_WDATA,s_axi_ctrl_WSTRB,s_axi_ctrl_ARVALID,s_axi_ctrl_ARREADY,s_axi_ctrl_ARADDR,s_axi_ctrl_RVALID,s_axi_ctrl_RREADY,s_axi_ctrl_RDATA,s_axi_ctrl_RRESP,s_axi_ctrl_BVALID,s_axi_ctrl_BREADY,s_axi_ctrl_BRESP,interrupt);
input   ap_clk;
input   ap_rst_n;
input  [23:0] image_in_TDATA;
input   image_in_TVALID;
output   image_in_TREADY;
output  [95:0] centers_out_TDATA;
output   centers_out_TVALID;
input   centers_out_TREADY;
input   s_axi_ctrl_AWVALID;
output   s_axi_ctrl_AWREADY;
input  [4:0] s_axi_ctrl_AWADDR;
input   s_axi_ctrl_WVALID;
output   s_axi_ctrl_WREADY;
input  [31:0] s_axi_ctrl_WDATA;
input  [3:0] s_axi_ctrl_WSTRB;
input   s_axi_ctrl_ARVALID;
output   s_axi_ctrl_ARREADY;
input  [4:0] s_axi_ctrl_ARADDR;
output   s_axi_ctrl_RVALID;
input   s_axi_ctrl_RREADY;
output  [31:0] s_axi_ctrl_RDATA;
output  [1:0] s_axi_ctrl_RRESP;
output   s_axi_ctrl_BVALID;
input   s_axi_ctrl_BREADY;
output  [1:0] s_axi_ctrl_BRESP;
output   interrupt;
endmodule
