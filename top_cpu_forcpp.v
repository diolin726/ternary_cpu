`include "cpu.v"
module top_cpu(input [ 32768  -1 : 0  ] input_code ,input [1:0] rst_n ,input clk );
    genvar i;
    //IF
    wire [41:0] ter_pc_IF ;
    wire [31:0]  readonlymem_IF;
    IF IF_top_cpu( .stop(stop_ctrl) ,.clk(clk) ,.input_code(input_code) ,.rst_n(rst_n) ,.pc_wr_en(pc_wr_en_out_mem) ,.pc_addr(pc_wr_addr_mem) ,.ter_pc(ter_pc_IF) ,.readonlymem(readonlymem_IF));
    //end IF
    // IF-ID
    reg [31:0] readonlymem_reg_IF_stage;
    reg [31:0] readonlymem_reg_IF;
    always @(posedge clk ) begin
        if (rst_n == 2'b00) begin
            readonlymem_reg_IF_stage <= 32'b0;
            readonlymem_reg_IF       <= 32'b0;
        end else begin
            // first stage samples combinational readonlymem_IF
            readonlymem_reg_IF_stage <= readonlymem_IF;
            // second stage produces the delayed-by-one-cycle value used by ID
            readonlymem_reg_IF       <= readonlymem_reg_IF_stage;        
        end
    end
    wire [41:0] ter_pc_IF_reg;
    terdff_to terdff_IF_ID_ter_pc( .clk(clk ) ,.rst_n(rst_n) ,.d(ter_pc_IF) ,.q(ter_pc_IF_reg) ); 
    // end IF-ID
    //ID
    wire [7:0] rs1_ID, rs2_ID ,rd_ID;
    wire [41:0] pc_ID , imm_ID;
    wire [5:0] func3_ID;
    wire [11:0]func7_ID; 
    wire [9:0] opcode_ID;
    ID ID_top_cpu( 
        .readonlymem(readonlymem_reg_IF),
        .pre_pc(ter_pc_IF_reg),
        .stop(stop_ctrl),
        .rs1(rs1_ID), //to ctrl unit 
        .next_pc( pc_ID ),
        .rs2(rs2_ID), //to ctrl unit 
        .rd(rd_ID), 
        .imm(imm_ID),
        .func3( func3_ID), 
        .func7(func7_ID), 
        .opcode(opcode_ID)
        );
    wire [41:0] rs1_data_r_w_reg , rs2_data_r_w_reg;
    read_write_reg r_w_reg_top_cpu(
        .rs1_fow_en(rs1_fow_en_ctrl) ,
        .rs2_fow_en(rs2_fow_en_ctrl) ,
        .rs1_fowarding(rs1_fowarding_ctrl) , 
        .rs2_fowarding(rs2_fowarding_ctrl) ,  
        .clk(clk),
        .rst_n(rst_n) ,
        .rs1(rs1_ID),
        .rs2(rs2_ID), 
        .wr_reg_addr(rd_out_wb  ) ,
        .wr_data(data_out_wb) ,
        .reg_wr_en( reg_wr_en_wb), 
        .rs1_data( rs1_data_r_w_reg),
        .rs2_data( rs2_data_r_w_reg) 
        );      
    //end ID
    //ID-EX
    wire [41:0] pc_ID_reg , imm_ID_reg , rs1_data_r_w_reg_reg , rs2_data_r_w_reg_reg ;
    wire [7:0] rd_ID_reg;// 4 + 3 + 6 + 5
    wire [5:0] func3_ID_reg;
    wire [11:0] func7_ID_reg;
    wire [9:0] opcode_ID_reg;
    terdff_to terdff_to_ID_EX_pc_ID_reg( .clk(clk ) ,.rst_n(rst_n ) ,.d( pc_ID) ,.q(pc_ID_reg));
    terdff_to terdff_to_ID_EX_imm_ID_reg( .clk(clk ) ,.rst_n(rst_n ) ,.d( imm_ID) ,.q(imm_ID_reg));
    terdff_to terdff_to_ID_EX_rs1_data_r_w_reg_reg( .clk(clk ) ,.rst_n(rst_n ) ,.d( rs1_data_r_w_reg) ,.q(rs1_data_r_w_reg_reg));
    terdff_to terdff_to_ID_EX_rs2_data_r_w_reg_reg( .clk(clk ) ,.rst_n(rst_n  ),.d( rs2_data_r_w_reg) ,.q(rs2_data_r_w_reg_reg));
    wire [35:0] ID_EX_reg_tmp , ID_EX_reg_tmp_reg;
    assign ID_EX_reg_tmp = { rd_ID , func3_ID  , func7_ID , opcode_ID};
    generate
        for(i=0 ;i < 18; i = i+1)begin : ID_EX_reg_array
            terdff terdff_ID_EX_ID_EX_reg_tmp_reg( .d( ID_EX_reg_tmp[i*2+1 : i*2]) ,.q(ID_EX_reg_tmp_reg[i*2+1 :i*2 ]) ,.clk(clk) ,.rst_n(rst_n) );
        end
    endgenerate
    assign { rd_ID_reg , func3_ID_reg  , func7_ID_reg , opcode_ID_reg } = ID_EX_reg_tmp_reg;
    //end ID-EX
    //EX
    wire [ 7:0 ]next_rd_EX; 
    wire [ 1:0 ]pc_wr_en_EX,  type_EX , read_sign_EX; 
    wire [41:0] data_EX , wr_data_EX;
    wire [3:0] read_wr_len_EX;

    EX EX_top_cpu(
        .pre_pc( pc_ID_reg),
        .pre_rd(rd_ID_reg), 
        .imm( imm_ID_reg),
        .func3(func3_ID_reg), 
        .func7(func7_ID_reg), 
        .opcode(opcode_ID_reg),
        .rs1_data(rs1_data_r_w_reg_reg),
        .rs2_data(rs2_data_r_w_reg_reg),
        .EX_change(EX_change_ctrl),//if 1 , rd = 0 type = T 
        .next_rd(  next_rd_EX),
        .pc_wr_en( pc_wr_en_EX),
        .type_EX(type_EX),  // T ( write rd ) 0( read mem write rd ) 1( write mem )
        .data(data_EX),
        .wr_data(wr_data_EX) ,
        .read_wr_len( read_wr_len_EX ),
        .read_sign( read_sign_EX )
        );
    //end EX
    //EX-mem
    wire [ 7:0 ]next_rd_EX_reg; // 4  + 3 + 2
    wire [ 1:0 ]pc_wr_en_EX_reg,  type_EX_reg , read_sign_EX_reg; 
    wire [41:0] data_EX_reg , wr_data_EX_reg;
    wire [3:0] read_wr_len_EX_reg;
    wire [17:0] EX_mem_reg_tmp , EX_mem_reg_tmp_reg;
    terdff_to terdff_to_data_EX_reg( .q( data_EX_reg) ,.d( data_EX) ,.clk(clk ) ,.rst_n(rst_n));
    terdff_to terdff_to_wr_data_EX_reg( .q( wr_data_EX_reg) ,.d( wr_data_EX) ,.clk(clk ) ,.rst_n(rst_n));
    assign EX_mem_reg_tmp = { next_rd_EX , pc_wr_en_EX , type_EX , read_sign_EX , read_wr_len_EX  };
    generate
        for(i=0 ;i < 9 ; i = i+1)begin : EX_mem_reg_array
            terdff terdff_EX_mem_EX_mem_reg_tmp_reg( .d( EX_mem_reg_tmp[i*2+1 : i*2]) ,.q(EX_mem_reg_tmp_reg[i*2+1 :i*2 ]) ,.clk(clk) ,.rst_n(rst_n) );
        end
    endgenerate
    assign { next_rd_EX_reg , pc_wr_en_EX_reg , type_EX_reg , read_sign_EX_reg , read_wr_len_EX_reg } = EX_mem_reg_tmp_reg;
    //end EX-mem
    //mem
    wire [1:0] pc_wr_en_out_mem;
    wire [41:0] data_out_mem , pc_wr_addr_mem;
    wire [ 7:0] rd_out_mem;
    mem mem_top_cpu( 
        .next_rd(next_rd_EX_reg),
        .pc_wr_en(pc_wr_en_EX_reg),
        .type_mem(type_EX_reg),  // T ( write rd ) 0( read mem write rd ) 1( write mem )
        .data(data_EX_reg),
        .wr_data(wr_data_EX_reg),
        .read_wr_len(read_wr_len_EX_reg), //b op , h pn , w pp
        .read_sign(read_sign_EX_reg), // p = unsign
        .clk(clk),
        .pc_wr_en_out( pc_wr_en_out_mem),
        .data_out(data_out_mem), // type o = mem , T = data ;
        .pc_wr_addr(pc_wr_addr_mem),//data 
        .rd_out(rd_out_mem )//next_rd
        );
    //end mem
    //mem-wb
    wire [41:0] data_out_mem_reg ;
    wire [ 7:0] rd_out_mem_reg;
    terdff_to terdff_to_data_out_mem_reg( .q( data_out_mem_reg) ,.d( data_out_mem) ,.clk(clk ) ,.rst_n(rst_n));
    wire [7:0] mem_wb_reg_tmp ,mem_wb_reg_tmp_reg;
    assign mem_wb_reg_tmp =  rd_out_mem;
    generate
        for(i=0 ;i < 4 ; i = i+1)begin : mem_wb_reg_array
            terdff terdff_mem_wb_mem_wb_reg_tmp_reg( .d(mem_wb_reg_tmp[i*2+1 : i*2]) ,.q(mem_wb_reg_tmp_reg[i*2+1 :i*2 ]) ,.clk(clk) ,.rst_n(rst_n) );
        end
    endgenerate
    assign rd_out_mem_reg  = mem_wb_reg_tmp_reg;
    //end mem-wb
    //wb
    wire [7:0] rd_out_wb;
    wire [41:0] data_out_wb;
    wire [1:0] reg_wr_en_wb;
    wb wb_top_cpu(
        .data( data_out_mem_reg),
        .rd(  rd_out_mem_reg  ),
        .data_out(data_out_wb),
        .rd_out( rd_out_wb), 
        .reg_wr_en( reg_wr_en_wb)
        );
    //end wb
    //ctrl
    wire [1:0] stop_ctrl ,EX_change_ctrl;  
    wire [1:0] rs1_fow_en_ctrl,rs2_fow_en_ctrl  ; 
    wire [41:0]  rs1_fowarding_ctrl ,  rs2_fowarding_ctrl;
    ctrl ctrl_top_cpu(
        .pc_wr_en( pc_wr_en_out_mem),
        .rd_EX(next_rd_EX),
        .rd_mem( rd_out_mem),
        .rd_wb(rd_out_wb),
        .rd_fowarding_data_mem(data_out_mem),
        .rd_fowarding_data_wb( data_out_wb),
        .rs1(rs1_ID),
        .rs2(rs2_ID),
        .stop( stop_ctrl )/*if stop , id addi x0, x0, 0 */,
        .EX_change(EX_change_ctrl)/* EX rd = 0 , type = T */,
        .rs1_fowarding(rs1_fowarding_ctrl),
        .rs2_fowarding(rs2_fowarding_ctrl),
        .rs1_fow_en(rs1_fow_en_ctrl),
        .rs2_fow_en(rs2_fow_en_ctrl)
        );
endmodule

