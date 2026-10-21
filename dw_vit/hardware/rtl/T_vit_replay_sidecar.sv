// USB replay sidecar. No sensor pixel or camera control outputs.
// Private commands are checksum-validated by the existing AA55 decoder.
module T_vit_replay_sidecar(input wire clk, input wire rst_n,
 input wire prot_valid, input wire [15:0] prot_addr,prot_data,
 input wire tx_busy, output wire tx_en,tx_valid,output wire [31:0] tx_word);
wire [31:0] a_awaddr;
wire [7:0] a_awlen;
wire [2:0] a_awsize;
wire [1:0] a_awburst;
wire a_awid;
wire a_awvalid;
wire a_awready;
wire [31:0] a_wdata;
wire [3:0] a_wstrb;
wire a_wlast;
wire a_wvalid;
wire a_wready;
wire a_bid;
wire [1:0] a_bresp;
wire a_bvalid;
wire a_bready;
wire [31:0] a_araddr;
wire [7:0] a_arlen;
wire [2:0] a_arsize;
wire [1:0] a_arburst;
wire a_arid;
wire a_arvalid;
wire a_arready;
wire [31:0] a_rdata;
wire [1:0] a_rresp;
wire a_rid;
wire a_rlast;
wire a_rvalid;
wire a_rready;
wire [4:0] c_awaddr;
wire c_awvalid;
wire c_awready;
wire [31:0] c_wdata;
wire [3:0] c_wstrb;
wire c_wvalid;
wire c_wready;
wire [1:0] c_bresp;
wire c_bvalid;
wire c_bready;
wire [4:0] c_araddr;
wire c_arvalid;
wire c_arready;
wire [31:0] c_rdata;
wire [1:0] c_rresp;
wire c_rvalid;
wire c_rready;
wire core_rst_n,image_valid,image_ready,center_valid,center_ready;
wire [23:0] image_data;wire [95:0] center_data;wire algorithm_interrupt;
T_usb_axi_master u_usb_master (
 .clk(clk),
 .rst_n(rst_n),
 .prot_valid(prot_valid),.prot_addr(prot_addr),.prot_data(prot_data),
 .tx_busy(tx_busy),.tx_en(tx_en),.tx_valid(tx_valid),.tx_word(tx_word),
 .m_axi_awaddr(a_awaddr),
 .m_axi_awlen(a_awlen),
 .m_axi_awsize(a_awsize),
 .m_axi_awburst(a_awburst),
 .m_axi_awid(a_awid),
 .m_axi_awvalid(a_awvalid),
 .m_axi_awready(a_awready),
 .m_axi_wdata(a_wdata),
 .m_axi_wstrb(a_wstrb),
 .m_axi_wlast(a_wlast),
 .m_axi_wvalid(a_wvalid),
 .m_axi_wready(a_wready),
 .m_axi_bid(a_bid),
 .m_axi_bresp(a_bresp),
 .m_axi_bvalid(a_bvalid),
 .m_axi_bready(a_bready),
 .m_axi_araddr(a_araddr),
 .m_axi_arlen(a_arlen),
 .m_axi_arsize(a_arsize),
 .m_axi_arburst(a_arburst),
 .m_axi_arid(a_arid),
 .m_axi_arvalid(a_arvalid),
 .m_axi_arready(a_arready),
 .m_axi_rdata(a_rdata),
 .m_axi_rresp(a_rresp),
 .m_axi_rid(a_rid),
 .m_axi_rlast(a_rlast),
 .m_axi_rvalid(a_rvalid),
 .m_axi_rready(a_rready)
);
T_vit_replay_bridge u_bridge (
 .algorithm_interrupt(algorithm_interrupt),
 .clk(clk),
 .rst_n(rst_n),
 .awaddr(a_awaddr),
 .awlen(a_awlen),
 .awsize(a_awsize),
 .awburst(a_awburst),
 .awid(a_awid),
 .awvalid(a_awvalid),
 .awready(a_awready),
 .wdata(a_wdata),
 .wstrb(a_wstrb),
 .wlast(a_wlast),
 .wvalid(a_wvalid),
 .wready(a_wready),
 .bid(a_bid),
 .bresp(a_bresp),
 .bvalid(a_bvalid),
 .bready(a_bready),
 .araddr(a_araddr),
 .arlen(a_arlen),
 .arsize(a_arsize),
 .arburst(a_arburst),
 .arid(a_arid),
 .arvalid(a_arvalid),
 .arready(a_arready),
 .rdata(a_rdata),
 .rresp(a_rresp),
 .rid(a_rid),
 .rlast(a_rlast),
 .rvalid(a_rvalid),
 .rready(a_rready),
 .c_awaddr(c_awaddr),
 .c_awvalid(c_awvalid),
 .c_awready(c_awready),
 .c_wdata(c_wdata),
 .c_wstrb(c_wstrb),
 .c_wvalid(c_wvalid),
 .c_wready(c_wready),
 .c_bresp(c_bresp),
 .c_bvalid(c_bvalid),
 .c_bready(c_bready),
 .c_araddr(c_araddr),
 .c_arvalid(c_arvalid),
 .c_arready(c_arready),
 .c_rdata(c_rdata),
 .c_rresp(c_rresp),
 .c_rvalid(c_rvalid),
 .c_rready(c_rready),
 .core_rst_n(core_rst_n),
 .image_data(image_data),
 .image_valid(image_valid),
 .image_ready(image_ready),
 .center_data(center_data),
 .center_valid(center_valid),
 .center_ready(center_ready)
);
vitseg_core u_algorithm (
 .ap_clk(clk),
 .ap_rst_n(core_rst_n),
 .s_axi_ctrl_AWADDR(c_awaddr),
 .s_axi_ctrl_AWVALID(c_awvalid),
 .s_axi_ctrl_AWREADY(c_awready),
 .s_axi_ctrl_WDATA(c_wdata),
 .s_axi_ctrl_WSTRB(c_wstrb),
 .s_axi_ctrl_WVALID(c_wvalid),
 .s_axi_ctrl_WREADY(c_wready),
 .s_axi_ctrl_BRESP(c_bresp),
 .s_axi_ctrl_BVALID(c_bvalid),
 .s_axi_ctrl_BREADY(c_bready),
 .s_axi_ctrl_ARADDR(c_araddr),
 .s_axi_ctrl_ARVALID(c_arvalid),
 .s_axi_ctrl_ARREADY(c_arready),
 .s_axi_ctrl_RDATA(c_rdata),
 .s_axi_ctrl_RRESP(c_rresp),
 .s_axi_ctrl_RVALID(c_rvalid),
 .s_axi_ctrl_RREADY(c_rready),
 .image_in_TDATA(image_data),
 .image_in_TVALID(image_valid),
 .image_in_TREADY(image_ready),
 .centers_out_TDATA(center_data),
 .centers_out_TVALID(center_valid),
 .centers_out_TREADY(center_ready),
 .interrupt(algorithm_interrupt)
);
endmodule
