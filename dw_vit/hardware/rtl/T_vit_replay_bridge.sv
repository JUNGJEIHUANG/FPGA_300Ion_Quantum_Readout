// Single-outstanding AXI4 slave. Low 32 bytes transparently access HLS AXI-Lite.
// 0x100: ID; 0x104: reset (write 1); 0x108..11c: counters.
// 0x1000..1fff: input FIFO write aperture, 18-bit raw ap_fixed<18,10>.
// 0x2000..2fff: output FIFO read aperture, low/middle/high 32-bit words.
// Camera-preserving replay sidecar; it does not drive camera control or image buses.
// Host must drain output while polling, not wait for ap_done with a full FIFO.
// FIFOs use distributed RAM. Signature VIT1 prevents confusing the old DW harness.
module T_vit_replay_bridge (
 input wire clk, rst_n,
 input wire algorithm_interrupt,
 input wire [31:0] awaddr, input wire [7:0] awlen, input wire [2:0] awsize,
 input wire [1:0] awburst, input wire awid, awvalid, output wire awready,
 input wire [31:0] wdata, input wire [3:0] wstrb, input wire wlast,wvalid,
 output wire wready, output wire bid, output wire [1:0] bresp,
 output wire bvalid, input wire bready,
 input wire [31:0] araddr, input wire [7:0] arlen, input wire [2:0] arsize,
 input wire [1:0] arburst, input wire arid, arvalid, output wire arready,
 output wire [31:0] rdata, output wire [1:0] rresp, output wire rid,rlast,rvalid,
 input wire rready,
 output wire core_rst_n,
 output wire [23:0] image_data, output wire image_valid, input wire image_ready,
 input wire [95:0] center_data, input wire center_valid, output wire center_ready,
 output wire [4:0] c_awaddr, output wire c_awvalid, input wire c_awready,
 output wire [31:0] c_wdata, output wire [3:0] c_wstrb,
 output wire c_wvalid, input wire c_wready, input wire [1:0] c_bresp,
 input wire c_bvalid, output wire c_bready,
 output wire [4:0] c_araddr, output wire c_arvalid, input wire c_arready,
 input wire [31:0] c_rdata, input wire [1:0] c_rresp,
 input wire c_rvalid, output wire c_rready
);
 // Metadata fingerprint; build provenance must separately bind the actual ROM files.
 localparam [255:0] CHECKPOINT_SHA256=256'hcc4debf1d0e3ee926b7b429678bd78e7e3ebd60260e4de2519bbaf6cdb8a64cc;
 // Input domain is raw Q8 grayscale 0..256, exactly representable in 9 bits.
 // Stored activations/weights INSIDE vitseg_core are unchanged (A18/W8 hybrid).
 (* ram_style="distributed" *) reg [8:0] in_mem[0:40127];
 (* ram_style="distributed" *) reg [95:0] out_mem[0:511];
 reg [15:0] iw,ir;
 reg [9:0] ow,orr;
 wire [15:0] in_used=iw-ir;
 wire [9:0] out_used=ow-orr;
 reg [31:0] n_in,n_consume,n_out,n_read,input_signature;
 reg [1:0] word_phase;
 reg [4:0] reset_count;
 reg reset_pulse;
 assign core_rst_n=rst_n && (reset_count==0);
 always @(posedge clk) begin
   if(!rst_n) reset_count<=31;
   else if(reset_pulse) reset_count<=16;
   else if(reset_count!=0) reset_count<=reset_count-1'b1;
 end
 assign image_data={15'b0,in_mem[ir]};
 assign image_valid=core_rst_n && timing_started && n_in==40128 && ir<40128;
 assign center_ready=core_rst_n && out_used<512 && pause_remaining==0;


 // All timestamps are 100 MHz cycles, private-reset epoch. No host poll timing.
 // IRQ is ap_done delayed by two cycles in generated HLS AXI-Lite control.
 reg [63:0] tick,start_tick,first_in_tick,last_in_tick,irq_tick,first_out_tick,last_out_tick;
 reg timing_started,timing_done;
 reg [63:0] output_stall_cycles,input_starvation_cycles;
 reg [31:0] pause_config,pause_remaining;
 always @(posedge clk) begin
   if(!core_rst_n) begin pause_config<=0;pause_remaining<=0;end
   else if(wvalid && wready && !wr_ctrl && wr_addr==32'h1c0) begin
     pause_config<=wdata;pause_remaining<=wdata;
   end else if(center_valid && pause_remaining!=0) pause_remaining<=pause_remaining-1;
 end
 wire start_event=c_wvalid && c_wready && c_awaddr==0 && c_wstrb[0] && c_wdata[0];
 always @(posedge clk) begin
   if(!core_rst_n) begin
     tick<=0;start_tick<=0;first_in_tick<=0;last_in_tick<=0;irq_tick<=0;
     first_out_tick<=0;last_out_tick<=0;timing_started<=0;timing_done<=0;
     output_stall_cycles<=0;input_starvation_cycles<=0;
   end else begin
     tick<=tick+1;
     if(timing_started && !timing_done && center_valid && !center_ready) output_stall_cycles<=output_stall_cycles+1;
     if(timing_started && !timing_done && n_consume<40128 && image_ready && !image_valid) input_starvation_cycles<=input_starvation_cycles+1;
     if(start_event && !timing_started) begin start_tick<=tick;timing_started<=1;end
     if(image_valid && image_ready) begin
       if(n_consume==0) first_in_tick<=tick;
       if(n_consume==40127) last_in_tick<=tick;
     end
     if(center_valid && center_ready) begin
       if(n_out==0) first_out_tick<=tick;
       last_out_tick<=tick;
     end
     if(timing_started && algorithm_interrupt && !timing_done) begin
       irq_tick<=tick;timing_done<=1;
     end
   end
 end
 reg wr_active,wr_ctrl,wr_ctrl_aw,wr_resp,wr_id,wr_bad;
 reg [31:0] wr_addr;
 reg [7:0] wr_left;
 reg [1:0] wr_burst;
 wire input_access=(wr_addr[31:12]==20'h00001);
 assign awready=!wr_active && !wr_resp;
 assign c_awaddr=wr_addr[4:0];
 assign c_awvalid=wr_active && wr_ctrl && wr_ctrl_aw && core_rst_n;
 // An early start is acknowledged with AXI error, but never starts the core.
 assign c_wdata=(wr_addr==0 && wdata[0] && n_in!=40128) ? (wdata & 32'hfffffffe) : wdata;
 assign c_wstrb=wstrb;
 assign c_wvalid=wr_active && wr_ctrl && !wr_ctrl_aw && !wr_resp && wvalid && core_rst_n;
 assign wready=wr_active && !wr_resp && (wr_ctrl ? (!wr_ctrl_aw && c_wready && core_rst_n) :
                   (input_access ? (iw<40128 && core_rst_n && !timing_started) : 1'b1));
 assign bvalid=wr_ctrl ? (wr_resp && c_bvalid) : wr_resp;
 assign bresp=wr_bad ? 2'b10 : (wr_ctrl ? c_bresp : 2'b00);
 assign bid=wr_id;
 assign c_bready=wr_ctrl && wr_resp && bready;
 wire push_in=wvalid && wready && !wr_ctrl && input_access && wstrb==4'hf && wdata<=256;
 always @(posedge clk) begin
   if(!rst_n) begin
     wr_active<=0;wr_ctrl<=0;wr_ctrl_aw<=0;wr_resp<=0;wr_id<=0;
     wr_bad<=0;wr_addr<=0;wr_left<=0;wr_burst<=0;reset_pulse<=0;
   end else begin
     reset_pulse<=0;
     if(awvalid && awready) begin
       wr_active<=1;wr_ctrl<=(awaddr[31:5]==0);wr_ctrl_aw<=1;
       wr_addr<=awaddr;wr_left<=awlen;wr_burst<=awburst;wr_id<=awid;
       wr_bad<=(awsize!=2 || awaddr[1:0]!=0 || awburst>1 ||
                ((awaddr[31:5]==0) && awlen!=0));
     end
     if(c_awvalid && c_awready) wr_ctrl_aw<=0;
     if(wvalid && wready) begin
       if(wlast!=(wr_left==0) || wstrb!=4'hf) wr_bad<=1;
       if(!wr_ctrl && !input_access && wr_addr!=32'h104 && wr_addr!=32'h1c0) wr_bad<=1;
       if(wr_ctrl && wr_addr==0 && wdata[0] && n_in!=40128) wr_bad<=1;
       if(input_access && wdata>256) wr_bad<=1;
       if(!wr_ctrl && wr_addr==32'h104 && wstrb[0] && wdata[0]) reset_pulse<=1;
       if(wlast || wr_left==0) begin wr_active<=0;wr_resp<=1;end
       else begin
         wr_left<=wr_left-1'b1;
         if(wr_burst==1) wr_addr<=wr_addr+4;
       end
     end
     if(bvalid && bready) wr_resp<=0;
   end
 end

 reg rd_active,rd_ctrl,rd_ctrl_ar,rd_id,rd_bad,rd_local_valid;
 reg [31:0] rd_addr,rd_local_data;
 reg [7:0] rd_left;
 reg [1:0] rd_burst;
 wire output_access=(rd_addr[31:12]==20'h00002);
 assign arready=!rd_active;
 assign c_araddr=rd_addr[4:0];
 assign c_arvalid=rd_active && rd_ctrl && rd_ctrl_ar && core_rst_n;
 assign c_rready=rd_active && rd_ctrl && !rd_ctrl_ar && rready;
 assign rvalid=rd_ctrl ? (rd_active && !rd_ctrl_ar && c_rvalid) : rd_local_valid;
 assign rdata=rd_ctrl ? c_rdata : rd_local_data;
 assign rresp=rd_bad ? 2'b10 : (rd_ctrl ? c_rresp : 2'b00);
 assign rid=rd_id;
 assign rlast=rd_left==0;
 wire pop_word=rvalid && rready && !rd_ctrl && output_access;
 wire pop_packet=pop_word && word_phase==2;
 always @(posedge clk) begin
   if(!rst_n) begin
     rd_active<=0;rd_ctrl<=0;rd_ctrl_ar<=0;rd_id<=0;rd_bad<=0;
     rd_local_valid<=0;rd_addr<=0;rd_left<=0;rd_burst<=0;rd_local_data<=0;
   end else begin
     if(arvalid && arready) begin
       rd_active<=1;rd_ctrl<=(araddr[31:5]==0);rd_ctrl_ar<=1;
       rd_addr<=araddr;rd_left<=arlen;rd_burst<=arburst;rd_id<=arid;
       rd_bad<=(arsize!=2 || araddr[1:0]!=0 || arburst>1 ||
               ((araddr[31:5]==0) && arlen!=0));
     end
     if(c_arvalid && c_arready) rd_ctrl_ar<=0;
     if(rd_active && !rd_ctrl && !rd_local_valid && (!output_access || out_used!=0)) begin
       rd_local_valid<=1;
       if(output_access) begin
         case(word_phase)
           0:rd_local_data<=out_mem[orr[8:0]][31:0];
           1:rd_local_data<=out_mem[orr[8:0]][63:32];
           2:rd_local_data<=out_mem[orr[8:0]][95:64];
         endcase
       end else case(rd_addr)
         32'h100:rd_local_data<=32'h56495431;
         32'h104:rd_local_data<={31'b0,!core_rst_n};
         32'h108:rd_local_data<={16'b0,in_used};
         32'h10c:rd_local_data<=n_in;
         32'h110:rd_local_data<=n_consume;
         32'h114:rd_local_data<=n_out;
         32'h118:rd_local_data<=n_read;
         32'h11c:rd_local_data<={22'b0,out_used};
         32'h120:rd_local_data<=input_signature;
         32'h124:rd_local_data<=32'h0812ffcc; // W8/A18 candidate; signed raw logit threshold=-52.
         32'h128:rd_local_data<=32'h50524c33; // PRL3: full image preload, no USB input after start
         32'h12c:rd_local_data<={29'b0,(n_in==40128),timing_done,timing_started};
         32'h130:rd_local_data<=40128;
         32'h134:rd_local_data<={31'b0,(n_in==40128)};
         32'h138:rd_local_data<=output_stall_cycles[31:0];
         32'h13c:rd_local_data<=output_stall_cycles[63:32];
         32'h1b0:rd_local_data<=input_starvation_cycles[31:0];
         32'h1b4:rd_local_data<=input_starvation_cycles[63:32];
         32'h1c0:rd_local_data<=pause_config;
         32'h1c4:rd_local_data<=pause_remaining;
         32'h180:rd_local_data<=start_tick[31:0];
         32'h184:rd_local_data<=start_tick[63:32];
         32'h188:rd_local_data<=first_in_tick[31:0];
         32'h18c:rd_local_data<=first_in_tick[63:32];
         32'h190:rd_local_data<=last_in_tick[31:0];
         32'h194:rd_local_data<=last_in_tick[63:32];
         32'h198:rd_local_data<=irq_tick[31:0];
         32'h19c:rd_local_data<=irq_tick[63:32];
         32'h1a0:rd_local_data<=first_out_tick[31:0];
         32'h1a4:rd_local_data<=first_out_tick[63:32];
         32'h1a8:rd_local_data<=last_out_tick[31:0];
         32'h1ac:rd_local_data<=last_out_tick[63:32];
         32'h140:rd_local_data<=CHECKPOINT_SHA256[255:224];
         32'h144:rd_local_data<=CHECKPOINT_SHA256[223:192];
         32'h148:rd_local_data<=CHECKPOINT_SHA256[191:160];
         32'h14c:rd_local_data<=CHECKPOINT_SHA256[159:128];
         32'h150:rd_local_data<=CHECKPOINT_SHA256[127:96];
         32'h154:rd_local_data<=CHECKPOINT_SHA256[95:64];
         32'h158:rd_local_data<=CHECKPOINT_SHA256[63:32];
         32'h15c:rd_local_data<=CHECKPOINT_SHA256[31:0];
         default:begin rd_local_data<=32'hbad0add0;rd_bad<=1;end
       endcase
     end
     if(rvalid && rready) begin
       rd_local_valid<=0;
       if(rd_left==0) rd_active<=0;
       else begin
         rd_left<=rd_left-1'b1;
         if(rd_burst==1) rd_addr<=rd_addr+4;
       end
     end
   end
 end

 always @(posedge clk) begin
   if(!core_rst_n) begin
     iw<=0;ir<=0;ow<=0;orr<=0;word_phase<=0;
     n_in<=0;n_consume<=0;n_out<=0;n_read<=0;
     input_signature<=32'h00000001;
   end else begin
     if(push_in) begin in_mem[iw]<=wdata[8:0];iw<=iw+1'b1;n_in<=n_in+1;end
     if(image_valid && image_ready) begin
       ir<=ir+1'b1;n_consume<=n_consume+1;
       input_signature<={input_signature[26:0],input_signature[31:27]}^{14'b0,image_data[17:0]};
     end
     if(center_valid && center_ready) begin out_mem[ow[8:0]]<=center_data;ow<=ow+1'b1;n_out<=n_out+1;end
     if(pop_word) begin
       n_read<=n_read+1;
       if(word_phase==2) word_phase<=0;
       else word_phase<=word_phase+1'b1;
     end
     if(pop_packet) orr<=orr+1'b1;
   end
 end
endmodule
