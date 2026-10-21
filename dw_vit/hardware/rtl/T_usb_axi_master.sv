// AA55 private FPGA register commands. One USB request outstanding at a time.
// 7f10/11 AXI address lo/hi; 7f12/13 write data lo/hi; 7f14 count (1..256);
// 7f15 sequence; 8000..80ff = sequential 16-bit Q8 pixel payload;
// 7f00 opcode: 1=single write, 2=read count words, 3=write count input pixels.
// Request command is always LAST in the checksum-validated FPGA packet.
// Response: 31525554, sequence<<16|status, count, payload[count], additive checksum.
// Does not modify sensor, DDR, clocks, power or the official decoder.
module T_usb_axi_master (
 input wire clk,rst_n,prot_valid,input wire [15:0] prot_addr,prot_data,
 input wire tx_busy,output reg tx_en,tx_valid,output reg [31:0] tx_word,
 output wire [31:0] m_axi_awaddr,output wire [7:0] m_axi_awlen,
 output wire [2:0] m_axi_awsize,output wire [1:0] m_axi_awburst,
 output wire m_axi_awid,output wire m_axi_awvalid,input wire m_axi_awready,
 output wire [31:0] m_axi_wdata,output wire [3:0] m_axi_wstrb,
 output wire m_axi_wlast,m_axi_wvalid,input wire m_axi_wready,
 input wire m_axi_bid,input wire [1:0] m_axi_bresp,input wire m_axi_bvalid,
 output wire m_axi_bready,output wire [31:0] m_axi_araddr,
 output wire [7:0] m_axi_arlen,output wire [2:0] m_axi_arsize,
 output wire [1:0] m_axi_arburst,output wire m_axi_arid,m_axi_arvalid,
 input wire m_axi_arready,input wire [31:0] m_axi_rdata,
 input wire [1:0] m_axi_rresp,input wire m_axi_rid,m_axi_rlast,m_axi_rvalid,
 output wire m_axi_rready
);
 localparam IDLE=0,AW=1,W=2,B=3,AR=4,R=5,ARM=6,HEADER=7,SEQ=8,LEN=9,DATA=10,SUM=11,END=12;
 reg [3:0] state;
 reg [31:0] address,write_data,checksum;
 reg [15:0] sequence_id,response_sequence;
 reg [8:0] length,loaded,index,response_length;
 reg [7:0] status;
 reg burst_pixels,bad_payload;
 (* ram_style="distributed" *) reg [15:0] pixel_mem[0:255];
 (* ram_style="distributed" *) reg [31:0] response_mem[0:255];
 assign m_axi_awaddr=burst_pixels ? 32'h1000 : address;
 assign m_axi_awlen=burst_pixels ? length-1'b1 : 0;
 assign m_axi_awsize=2;assign m_axi_awburst=1;assign m_axi_awid=0;
 assign m_axi_awvalid=state==AW;
 assign m_axi_wdata=burst_pixels ? {16'b0,pixel_mem[index[7:0]]} : write_data;
 assign m_axi_wstrb=4'hf;assign m_axi_wlast=!burst_pixels || index+1==length;
 assign m_axi_wvalid=state==W;assign m_axi_bready=state==B;
 assign m_axi_araddr=address;assign m_axi_arlen=length-1'b1;
 assign m_axi_arsize=2;assign m_axi_arburst=1;assign m_axi_arid=0;
 assign m_axi_arvalid=state==AR;assign m_axi_rready=state==R;
 always @(posedge clk) begin
   if(!rst_n) begin
     state<=IDLE;address<=0;write_data<=0;sequence_id<=0;response_sequence<=0;
     length<=1;loaded<=0;index<=0;response_length<=0;status<=0;
     burst_pixels<=0;bad_payload<=0;tx_en<=0;tx_valid<=0;tx_word<=0;checksum<=0;
   end else begin
     tx_valid<=0;
     if(prot_valid && state==IDLE) begin
       if(prot_addr[15:8]==8'h80) begin
         if(prot_addr[7:0]==0) begin
           loaded<=1;bad_payload<=prot_data>256;pixel_mem[0]<=prot_data;
         end else begin
           if(loaded!={1'b0,prot_addr[7:0]} || prot_data>256) bad_payload<=1;
           pixel_mem[prot_addr[7:0]]<=prot_data;loaded<=loaded+1;
         end
       end else case(prot_addr)
         16'h7f10:address[15:0]<=prot_data;
         16'h7f11:address[31:16]<=prot_data;
         16'h7f12:write_data[15:0]<=prot_data;
         16'h7f13:write_data[31:16]<=prot_data;
         16'h7f14:length<=prot_data[8:0];
         16'h7f15:sequence_id<=prot_data;
         16'h7f00:begin
           index<=0;response_length<=0;status<=0;response_sequence<=sequence_id;
           if(tx_busy) begin status<=8'h04;state<=ARM;end
           else if(prot_data==1 && address[1:0]==0) begin burst_pixels<=0;state<=AW;end
           else if(prot_data==2 && length>=1 && length<=256 && address[1:0]==0) state<=AR;
           else if(prot_data==3 && length>=1 && length<=256 && loaded==length && !bad_payload) begin
             burst_pixels<=1;state<=AW;
           end else begin status<=8'h01;state<=ARM;end
         end
         default:;
       endcase
     end
     case(state)
       IDLE:tx_en<=0;
       AW:if(m_axi_awready) state<=W;
       W:if(m_axi_wready) begin
         if(m_axi_wlast) state<=B;
         else index<=index+1;
       end
       B:if(m_axi_bvalid) begin
         if(m_axi_bresp!=0) status<=8'h02;
         loaded<=0;state<=ARM;
       end
       AR:if(m_axi_arready) state<=R;
       R:if(m_axi_rvalid) begin
         response_mem[index[7:0]]<=m_axi_rdata;
         if(m_axi_rresp!=0 || m_axi_rlast!=(index+1==length)) status<=8'h03;
         if(index+1==length) begin response_length<=length;state<=ARM;end
         else index<=index+1;
       end
       ARM:if(!tx_busy) begin tx_en<=1;checksum<=0;index<=0;state<=HEADER;end
       HEADER:begin tx_valid<=1;tx_word<=32'h31525554;checksum<=32'h31525554;state<=SEQ;end
       SEQ:begin tx_valid<=1;tx_word<={response_sequence,8'b0,status};checksum<=checksum+{response_sequence,8'b0,status};state<=LEN;end
       LEN:begin tx_valid<=1;tx_word<={23'b0,response_length};checksum<=checksum+response_length;state<=response_length==0 ? SUM : DATA;end
       DATA:begin
         tx_valid<=1;tx_word<=response_mem[index[7:0]];checksum<=checksum+response_mem[index[7:0]];
         if(index+1==response_length) state<=SUM;else index<=index+1;
       end
       SUM:begin tx_valid<=1;tx_word<=checksum;state<=END;end
       END:begin tx_en<=0;state<=IDLE;end
       default:state<=IDLE;
     endcase
   end
 end
endmodule
