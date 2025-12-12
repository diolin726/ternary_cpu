
`include "basic.v"
/* verilator lint_off DECLFILENAME */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNOPTFLAT */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off CASEINCOMPLETE */

module pc(input clk , input [1:0]stop ,input [1:0]  rst_n ,input [1:0] pc_wr_en , input [ 41:0 ]pc_addr , output [31:0] pc_bi ,output [41:0]ter_pc );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [1:0] warn ,stop_o, pc1_choose ;
    terand terand_stop_o( .a( o) ,.b(stop) ,.out(stop_o));
    wire [41:0 ] next_add ,wire1,pc_next;
    //imm_bit_to_tit imm_bit_to_tit_pc(  .imm(pc_addr)  ,.ter_imm(pc_addr) );
    teror teror_pc1_choose( .out(pc1_choose) ,.a(stop_o ),.b( pc_wr_en) );
    baladder_to baladder_to_pc ( .a(ter_pc) ,.b({ {19{o}} , p,p}) , .cin( o ) , .cout(warn )  ,.sum(next_add) );
    to_mux to_mux_pc1( .a(  next_add   ) ,.b( ter_pc ) ,.c(  pc_addr  ) ,.choose( pc1_choose) ,.out( wire1 ) );
    to_mux to_mux_pc2(    .a( {21{o}} ) ,.b(  { 21{n}}  ) ,.c( wire1) ,.choose( rst_n ) ,.out(pc_next )  );
    terdff_to terdff_to_pc( .clk(clk) ,.d( pc_next ) ,.q(ter_pc) ,.rst_n( rst_n ));   
    baltit_to_bit baltit_to_bit_pc(  .in(ter_pc)  ,.out( pc_bi)    );
    //+892982539+296236+3626+2+985238936236+3+963++++++++36+.6+3++++++++++++++++++++6393.23926289852562299252962295252962jnhubibyygyyuyhuuuuuuuuuuuuuuuuuuuhih9i88777777y988888u98io0ooooooooooooooo0ui809i988
    //有低兒亂動我電腦
endmodule 
module rom( input [ 31:0 ] pc ,input [ 32768  -1 : 0  ] input_code , output [31 :0] readonlymem);// rom替代品 4kb
    assign readonlymem = input_code[  pc*8 +:32 ];
endmodule
module IF (  input clk,input [1:0]stop , input [ 32768  -1 : 0 ] input_code , input [1:0] rst_n , input [1:0] pc_wr_en , input [41:0] pc_addr  , output [41:0] ter_pc, output [31 :0] readonlymem);
    wire [31:0] pc_bi;
    wire [31:0] readonlymem_tmp;
    pc pc_IF(.stop(stop), .clk(clk ) ,.rst_n(rst_n) ,.pc_wr_en(pc_wr_en) ,.pc_addr(pc_addr ) ,.pc_bi(pc_bi )  ,.ter_pc(ter_pc) );
    rom rom_IF(.input_code(input_code) ,.pc(pc_bi) ,.readonlymem(readonlymem_tmp)  );
    //bin
    assign readonlymem = ( pc_wr_en == 2'b00)? readonlymem_tmp : 32'b00000000000000000000000000010011;
    //end bin
endmodule
module type_check( input [31:0]readonlymem  , output is_r ,  output is_i ,  output is_s,  output is_b,  output is_j,  output is_u ,output [4:0 ] opcode );
    //binary
    assign opcode = readonlymem[6:2];
    assign  is_r = opcode == 5'b01100,
            is_i = opcode == 5'b00100 || opcode == 5'b00000 || opcode  == 5'b11001, 
            is_s = opcode == 5'b01000,
            is_b = opcode == 5'b11000,
            is_u = opcode == 5'b01101 ||    opcode == 5'b00101,
            is_j = opcode == 5'b11011; 
endmodule
module ID(
    input [31:0]readonlymem,
    input [41:0]pre_pc,
    input [1:0]stop,
    output [7:0]rs1, //to ctrl unit 
    output [41:0] next_pc,
    output [7:0]rs2, //to ctrl unit 
    output [7:0]rd, 
    output [41 :0]imm,
    output [5:0] func3, 
    output [11:0]func7, 
    output [9:0] opcode
    );//rs1 , 2 data write another unitm
    //binary

    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire is_r , is_i ,is_s ,is_b , is_u , is_j;
    
    wire [4:0] opcode_bi , rs1_bi , rs2_bi ,rd_bi ;
    wire [2:0] func3_bi;
    wire [6:0] func7_bi ; 
    wire [ 31 : 0 ] imm_bi ;
    type_check type_check_ID( 
        .is_r(is_r ),    
        .is_i(is_i ),    
        .is_s(is_s ),    
        .is_b(is_b ),    
        .is_u(is_u ),    
        .is_j(is_j ),
        .opcode(opcode_bi ),
        .readonlymem(readonlymem)       
        );
    assign  rs1_bi = {5{rs1_en}} & readonlymem[19:15],
            rs2_bi = {5{rs2_en}} & readonlymem[24:20],
            func3_bi  =  readonlymem[14:12],
            rd_bi = rd_en ? readonlymem[11:7] : 5'b00000,
            func7_bi = func7_en? ( readonlymem[31:25]) : 7'd0;
    assign  imm_bi = (is_i)?{ {20{readonlymem[31]}} , readonlymem[31:20] }:
            (is_s)?{ {20{readonlymem[31]}} ,  readonlymem[31:25] , readonlymem[11:7] }:
            (is_b)?{ {20{ readonlymem[31]}} , readonlymem[7] ,readonlymem[30:25] ,readonlymem[11:8] ,1'b0 }:
            (is_u)?{ readonlymem[31:12] , 12'b0000_0000_0000   }:
            {  {12{ readonlymem[31] }}   , readonlymem[19:12 ] ,readonlymem[20] ,readonlymem[30:21]   ,1'b0 };
    //         //enable 等等改到執行裡
    // wire rs1_en ,rs2_en , rd_en , imm_en , func3_en , func7_en ; 
    // assign  func7_en = is_r,
    //         func3_en = rs1_en,
    //         rs1_en = is_r || is_i || is_b ||is_s ,
    //         rs2_en = is_r || is_b || is_s ,
    //         rd_en =  is_r || is_i || is_u || is_j ,
    //         imm_en = is_i || is_s || is_b || is_u || is_j ;
	// 		//end enable
    wire rd_en,func7_en ,rs2_en,rs1_en; 
    assign  rd_en =  is_r || is_i || is_u || is_j ,
            func7_en = is_r,
            rs1_en = is_r || is_i || is_b ||is_s ,
            rs2_en = is_r || is_b || is_s ;
    //ter
    wire [9:0] opcode_1;
    wire [7:0] rd_1;
    assign next_pc = pre_pc;
    five_bit_to_four_tit fbtft_ID_rs1 ( .in(rs1_bi) ,.out(rs1));
    five_bit_to_four_tit fbtft_ID_rs2 ( .in(rs2_bi) ,.out(rs2));
    five_bit_to_four_tit fbtft_ID_rd ( .in(rd_bi ) ,.out(rd_1));
	assign opcode_1 = { opcode_bi[4] ,1'b0 ,opcode_bi[3] ,1'b0 ,opcode_bi[2],1'b0  ,opcode_bi[1],1'b0  ,opcode_bi[0],1'b0   };
	three_bit_to_three_tit tbttt_ID_func3(  .in( func3_bi) ,.out( func3));
    seven_bit_to_six_tit sbtft_ID_func7(.in(func7_bi) ,.out(func7));
    imm_bit_to_tit ibtt_ID_imm(  .imm( imm_bi  ) ,.ter_imm( imm ));	
    //stop
    wire [1:0] stop_o;
    terand terand_ID_stop_o( .b(o),.a( stop) ,.out(stop_o));
    ternmul nmul_ID_rd_1( .a(stop_o ) ,.b(rd_1[1:0]) ,.out(rd[1:0]) );
    ternmul nmul_ID_rd_2( .a(stop_o ) ,.b(rd_1[3:2]) ,.out(rd[3:2]) );
    ternmul nmul_ID_rd_3( .a(stop_o ) ,.b(rd_1[5:4]) ,.out(rd[5:4]) );
    ternmul nmul_ID_rd_4( .a(stop_o ) ,.b(rd_1[7:6]) ,.out(rd[7:6]) );

    wire [1:0] stop_n;
    onot onot_ID_stop_n( .a( stop) ,.out(stop_n));
    terand terand_ID_opcode_1( .a(stop_n ) ,.b(opcode_1[1:0]) ,.out(opcode[1:0]) );
    terand teraand_ID_opcode_2( .a(stop_n ) ,.b(opcode_1[3:2]) ,.out(opcode[3:2]) );
    teror teror_ID_opcode_3( .a(stop) ,.b(opcode_1[5:4]) ,.out(opcode[5:4]));
    terand terand_ID_opcode_4( .a(stop_n ) ,.b(opcode_1[7:6]) ,.out(opcode[7:6]) );
    terand terand_ID_opcode_5( .a(stop_n ) ,.b(opcode_1[9:8]) ,.out(opcode[9:8]) );
endmodule
module read_write_reg (input [1:0]rs1_fow_en ,input [1:0]rs2_fow_en ,input [41:0]rs1_fowarding , input [41:0]rs2_fowarding ,  input clk ,input [1:0]rst_n ,input [7:0] rs1  ,input [7:0] rs2 , input [7:0] wr_reg_addr ,input [41:0] wr_data ,input [1:0] reg_wr_en, output [41:0] rs1_data ,output [41:0] rs2_data );      
    genvar i;
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [ 41:0 ]terreg [0:31]; 
    wire [ 41:0 ]terreg_in [0 : 31]; 
    wire [7:0] para [0:31];
    //assign parameter ;//cause i'm using iverilog , i've got a verlator though
    assign para[0]  = {o,o,o,o};
    assign para[1]  = {o,o,o,p};
    assign para[2]  = {o,o,p,n};
    assign para[3]  = {o,o,p,o};
    assign para[4]  = {o,o,p,p};
    assign para[5]  = {o,p,n,n};
    assign para[6]  = {o,p,n,o};
    assign para[7]  = {o,p,n,p};
    assign para[8]  = {o,p,o,n};
    assign para[9]  = {o,p,o,o};
    assign para[10] = {o,p,o,p};
    assign para[11] = {o,p,p,n};
    assign para[12] = {o,p,p,o};
    assign para[13] = {o,p,p,p};
    assign para[14] = {p,n,n,n};
    assign para[15] = {p,n,n,o};
    assign para[16] = {p,n,n,p};
    assign para[17] = {p,n,o,n};
    assign para[18] = {p,n,o,o};
    assign para[19] = {p,n,o,p};
    assign para[20] = {p,n,p,n};
    assign para[21] = {p,n,p,o};
    assign para[22] = {p,n,p,p};
    assign para[23] = {p,o,n,n};
    assign para[24] = {p,o,n,o};
    assign para[25] = {p,o,n,p};
    assign para[26] = {p,o,o,n};
    assign para[27] = {p,o,o,o};
    assign para[28] = {p,o,o,p};
    assign para[29] = {p,o,p,n};
    assign para[30] = {p,o,p,o};
    assign para[31] = {p,o,p,p};
    //end assign parameter
    wire [41:0] rs1_array[0:31];
    wire [41:0] rs2_array[0:31];
    wire [41:0] rs1_data_tmp , rs2_data_tmp;
    wire [1:0] rs1_fow_en_n , rs2_fow_en_n ; 
    onot onot_reg_rs1_fow_en_n( .a( rs1_fow_en) ,.out(rs1_fow_en_n));
    onot onot_reg_rs2_fow_en_n( .a( rs2_fow_en) ,.out(rs2_fow_en_n));
    generate
        for( i = 0 ; i < 32 ; i = i + 1 ) begin : reg_array
            //reg
            wire [1:0] change_this ;
            wire [41: 0 ] tmp ; 
            terdff_to terdff_to_read_write_reg( .d( terreg_in[i] ) ,.q(terreg[i]) ,.clk(clk ) , .rst_n(rst_n));
            four_eq four_eq_read_write_reg(.a(para[i]) ,.b(wr_reg_addr) ,.out(change_this));
            to_mux to_mux_read_write_reg1( .a( terreg[i] ) ,.b({21{n}}) ,.c(wr_data) ,.choose( reg_wr_en ) ,.out( tmp ));
            to_mux to_mux_read_write_reg2( .a( terreg[i] ) ,.b({21{n}}) ,.c(tmp) ,.choose( change_this ) ,.out(terreg_in[i]));
            //rs1 rs2
            wire [1:0] is_rs1 , is_rs2;      
            four_eq four_eq_rs1(.a(rs1)  ,.b(para[i]) ,.out( is_rs1));
            four_eq four_eq_rs2(.a(rs2) ,.b(para[i]) ,.out(is_rs2));
            to_mux to_mux_rs1(  .a( {21{n}} )  ,.b({21{n}})  ,.c(terreg[i]) ,.choose(is_rs1) ,.out(rs1_array[i]));
            to_mux to_mux_rs2( .a( {21{n}}) ,.b({21{n}}) ,.c(terreg[i]) ,.choose(is_rs2) ,.out(rs2_array[i]));
        end
    endgenerate
    generate
        for( i = 0 ; i < 21 ; i = i + 1 ) begin : rs1_rs2_assign
            //rs1
            wire [ 1 : 0 ] or_chain_rs1 [0:31];
            wire [ 1 : 0 ] or_chain_rs2 [0:31];
            wire [ 1 : 0 ] rs1_data_fow_tmp1 , rs1_data_fow_tmp2 ,rs2_data_fow_tmp1 , rs2_data_fow_tmp2  ;
            genvar ii;
            assign or_chain_rs2[0] = rs2_array[0][i*2 +1 : i*2 ];
            assign or_chain_rs1[0] = rs1_array[0][i*2 +1 : i*2 ];
            for( ii = 1 ;  ii < 32; ii = ii + 1 )begin : make_or_chain_rs1
               teror teror_reg_rs1( .a(or_chain_rs1[ii-1])  ,.b( rs1_array[ii][i*2 +1 : i*2])  ,.out(or_chain_rs1[ii]));
                teror teror_reg_rs2( .a(or_chain_rs2[ii-1])  ,.b( rs2_array[ii][i*2 +1 : i*2])  ,.out(or_chain_rs2[ii]));
            end
            assign rs1_data_tmp[i*2 +1 :i*2] = or_chain_rs1[31];
            assign rs2_data_tmp[i*2 +1 :i*2] = or_chain_rs2[31];
            terand terand_reg_rs1_data_tmp(  .a( rs1_data_tmp[i*2 +1 :i*2]) ,.b( rs1_fow_en_n) ,.out( rs1_data_fow_tmp1));
            terand terand_reg_rs1_fow_data(  .a( rs1_fowarding[i*2 +1 :i*2]) ,.b( rs1_fow_en ) ,.out( rs1_data_fow_tmp2));
            teror teror_reg_rs1_fow( .a( rs1_data_fow_tmp1) ,.b(rs1_data_fow_tmp2) ,.out( rs1_data[i*2+1 : i*2]) );

            terand terand_reg_rs2_data_tmp(  .a( rs2_data_tmp[i*2 +1 :i*2]) ,.b( rs2_fow_en_n) ,.out( rs2_data_fow_tmp1));
            terand terand_reg_rs2_fow_data(  .a( rs2_fowarding[i*2 +1 :i*2]) ,.b( rs2_fow_en ) ,.out( rs2_data_fow_tmp2));
            teror teror_reg_rs2_fow( .a( rs2_data_fow_tmp1) ,.b(rs2_data_fow_tmp2) ,.out( rs2_data[i*2+1 : i*2]) );
        end
    endgenerate
endmodule
//@ai , check by iverilog
module rsi_check(
    input  [9:0] in,   // 5 trits: in[9:8]=tr4 ... in[1:0]=tr0
    output [1:0] out   // p(group1) / n(group2) / o(group3)
);
    parameter p = 2'b10, n = 2'b00, o = 2'b01;

    wire [1:0] tr4 = in[9:8];
    wire [1:0] tr3 = in[7:6];
    wire [1:0] tr2 = in[5:4];
    wire [1:0] tr1 = in[3:2];
    wire [1:0] tr0 = in[1:0];

    // detect literals
    wire [1:0] tr4_p; isp isp_tr4(.a(tr4), .out(tr4_p));
    wire [1:0] tr3_p; isp isp_tr3(.a(tr3), .out(tr3_p));
    wire [1:0] tr2_p; isp isp_tr2(.a(tr2), .out(tr2_p));
    wire [1:0] tr1_p; isp isp_tr1(.a(tr1), .out(tr1_p));
    wire [1:0] tr0_p; isp isp_tr0(.a(tr0), .out(tr0_p));

    wire [1:0] tr4_n; isn isn_tr4(.a(tr4), .out(tr4_n));
    wire [1:0] tr3_n; isn isn_tr3(.a(tr3), .out(tr3_n));
    wire [1:0] tr2_n; isn isn_tr2(.a(tr2), .out(tr2_n));
    wire [1:0] tr1_n; isn isn_tr1(.a(tr1), .out(tr1_n));
    wire [1:0] tr0_n; isn isn_tr0(.a(tr0), .out(tr0_n));

    // intermediate top relations
    wire [1:0] top_pp; terand t_top_pp(.a(tr4_p), .b(tr3_p), .out(top_pp)); // both p
    wire [1:0] top_nn; terand t_top_nn(.a(tr4_n), .b(tr3_n), .out(top_nn)); // both n
    wire [1:0] top_np; terand t_top_np(.a(tr4_n), .b(tr3_p), .out(top_np)); // n,p

    // Group3 (unique): top_np && tr2==p && tr0==p  -> o
    wire [1:0] g3_m1; terand t_g3m1(.a(tr2_p), .b(tr0_p), .out(g3_m1));
    wire [1:0] g3_sel; terand t_g3(.a(top_np), .b(g3_m1), .out(g3_sel));

    // Group2 rules:
    // rule1: top_pp && tr1==p  -> group2
    wire [1:0] g2_r1; terand t_g2r1(.a(top_pp), .b(tr1_p), .out(g2_r1));
    // rule2: top_pp && tr0==n  -> group2
    wire [1:0] g2_r2; terand t_g2r2(.a(top_pp), .b(tr0_n), .out(g2_r2));
    // rule3: top_nn && tr2==p && tr0==p -> group2
    wire [1:0] g2_r3_m; terand t_g2r3m(.a(tr2_p), .b(tr0_p), .out(g2_r3_m));
    wire [1:0] g2_r3; terand t_g2r3(.a(top_nn), .b(g2_r3_m), .out(g2_r3));

    // combine group2 rules
    wire [1:0] g2_tmp; teror t_g2a(.a(g2_r1), .b(g2_r2), .out(g2_tmp));
    wire [1:0] g2_sel; teror t_g2b(.a(g2_tmp), .b(g2_r3), .out(g2_sel));

    // Final outputs: Group3 -> o (highest), Group2 -> n, else Group1 -> p
    wire [1:0] out_o; terand gate_o(.a(g3_sel), .b(o), .out(out_o));
    wire [1:0] out_n; terand gate_n(.a(g2_sel), .b(n), .out(out_n));

    // default p when neither g3 nor g2
    wire [1:0] not_g3; onot ng3(.a(g3_sel), .out(not_g3));
    wire [1:0] not_g2; onot ng2(.a(g2_sel), .out(not_g2));
    wire [1:0] p_enable; terand en_p(.a(not_g3), .b(not_g2), .out(p_enable));
    wire [1:0] out_p; terand gate_p(.a(p_enable), .b(p), .out(out_p));

    // combine with priority: o > n > p
    wire [1:0] tmp; teror tor1(.a(out_o), .b(out_n), .out(tmp));
    teror tor2(.a(tmp), .b(out_p), .out(out));
endmodule
//end @ai
module EX (
    input [41:0] pre_pc,
    input [7:0] pre_rd, 
    input [41 :0] imm,
    input [5:0] func3, 
    input [11:0] func7, 
    input [9:0] opcode,
    input [41:0] rs1_data,
    input [41:0] rs2_data,
    input [1:0] EX_change,//if 1 , rd = 0 type = T 
    output [7:0] next_rd,
    output [1:0] pc_wr_en,
    output [1:0] type_EX,  // T ( write rd ) 0( read mem write rd ) 1( write mem )
    output [41:0] data,
    output [41:0] wr_data,
    output [3:0] read_wr_len,
    output [ 1:0 ] read_sign
    );
    parameter p = 2'b10, n = 2'b00, o = 2'b01;
    //pre_pc is now pc but pre_pc_for_rd have to be pc + 4 so i gotta make a adder
    wire [1: 0 ] pre_pc_for_rd_trash_cout;
    wire [41:0]pre_pc_for_rd; 
    baladder_to baladder_to_pre_pc_for_rd( .a(pre_pc) ,.b({ {19{o} }, p,p}) , .cin( o ) , .cout( pre_pc_for_rd_trash_cout )  ,.sum(pre_pc_for_rd) );
    /*
    when pc_wr_en , data = next_pc , wr_data = pc + 4  ,if/id/mem/wb input addi x0 , x0 , 0 ,type = T (in ctrl unit)
    when type = T, data = rd
    when type = 0, data = mem addr (32bit)
    when type = 1, data = mem addr (32bit) , wr_data = rs2_data;
    */
    wire [1:0]EX_change_o; 
    terand terand_EX_EX_change_o( .a( o) ,.b( EX_change),.out(EX_change_o) );
    ternmul nmul_EX_next_rd1( .a(EX_change_o) ,.b( pre_rd[1:0]) ,.out(next_rd[1:0]));
    ternmul nmul_EX_next_rd2( .a(EX_change_o) ,.b( pre_rd[3:2]) ,.out(next_rd[3:2]));
    ternmul nmul_EX_next_rd3( .a(EX_change_o) ,.b( pre_rd[5:4]) ,.out(next_rd[5:4]));
    ternmul nmul_EX_next_rd4( .a(EX_change_o) ,.b( pre_rd[7:6]) ,.out(next_rd[7:6]));

    to_mux to_mux_wr_data( .a( rs2_data ) ,.b({21{n}}) ,.c( pre_pc_for_rd ) ,.choose(pc_wr_en) ,.out( wr_data) ); //nop
    //opcode find type
    wire [ 1 : 0 ] type_tmp;
    wire [ 1 : 0 ] is_type1tmp [ 0 : 7 ] ;
    wire [ 1 : 0 ] is_type1;
    isn isn_EX_opcode_istype1_1( .a(opcode[ 9 : 8 ] ) ,.out( is_type1tmp[0] ));
    isp isp_EX_opcode_istype1_2( .a(opcode[ 7 : 6 ] ) ,.out( is_type1tmp[1] ));
    isn isn_EX_opcode_istype1_3( .a(opcode[ 5 : 4 ] ) ,.out( is_type1tmp[2] ));
    isn isn_EX_opcode_istype1_4( .a(opcode[ 3 : 2 ] ) ,.out( is_type1tmp[3] ));
    isn isn_EX_opcode_istype1_5( .a(opcode[ 1 : 0 ] ) ,.out( is_type1tmp[4] ));
    terand terand_EX_opcode_istype1_1( .a(is_type1tmp[0]) ,.b(is_type1tmp[1])  ,.out( is_type1tmp[5]) );
    terand terand_EX_opcode_istype1_2( .a(is_type1tmp[5]) ,.b(is_type1tmp[2])  ,.out( is_type1tmp[6]) );
    terand terand_EX_opcode_istype1_3( .a(is_type1tmp[6]) ,.b(is_type1tmp[3])  ,.out( is_type1tmp[7]) );
    terand terand_EX_opcode_istype1_4( .a(is_type1tmp[7]) ,.b(is_type1tmp[4])  ,.out( is_type1) );

    wire [ 1 : 0 ] is_type0tmp [ 0 : 8 ] ;
    wire [ 1 : 0 ] is_type0;
    isn isn_EX_opcode_istype0_1( .a(opcode[ 9 : 8 ] ) ,.out( is_type0tmp[0] ));
    isn isn_EX_opcode_istype0_2( .a(opcode[ 7 : 6 ] ) ,.out( is_type0tmp[1] ));
    isn isn_EX_opcode_istype0_3( .a(opcode[ 5 : 4 ] ) ,.out( is_type0tmp[2] ));
    isn isn_EX_opcode_istype0_4( .a(opcode[ 3 : 2 ] ) ,.out( is_type0tmp[3] ));
    isn isn_EX_opcode_istype0_5( .a(opcode[ 1 : 0 ] ) ,.out( is_type0tmp[4] ));
    terand terand_EX_opcode_istype0_1( .a(is_type0tmp[0]) ,.b(is_type0tmp[1])  ,.out( is_type0tmp[5]) );
    terand terand_EX_opcode_istype0_2( .a(is_type0tmp[5]) ,.b(is_type0tmp[2])  ,.out( is_type0tmp[6]) );
    terand terand_EX_opcode_istype0_3( .a(is_type0tmp[6]) ,.b(is_type0tmp[3])  ,.out( is_type0tmp[7]) );
    terand terand_EX_opcode_istype0_4( .a(is_type0tmp[7]) ,.b(is_type0tmp[4])  ,.out( is_type0tmp[8]) );
    terand terand_EX_opcode_istype0_turn0( .a(is_type0tmp[8]) , .b(o) , .out( is_type0 ));
    
    teror teror_EX_opcode_type( .a(is_type0) ,.b(is_type1) ,.out(type_tmp)); 
    
    wire [ 1 : 0 ] EX_change_n;
    onot onot_EX_EX_change_n( .a( EX_change) ,.out(EX_change_n));
    terand terand_EX_type( .a( type_tmp) ,.b( EX_change_n) ,.out(type_EX) );
    //end opcode find type
    //r type and i type 
    //n x p n n #### use rtypedata

    //n n n n n and
    //p p n n p also use addi 
    wire [ 41 : 0 ] rs2_data_i_r;
    wire [ 41 : 0 ] rtypedata ;//(與i type 混用) //n x p n n
    wire [ 41 : 0 ] rtypedata_tmpp , rtypedata_tmpo , rtypedata_tmpn;  
    to_mux to_mux_EX_rtypedata( .choose(func3[1:0]),.out(rtypedata)  ,.a( rtypedata_tmpn) ,.b( rtypedata_tmpo ) ,.c( rtypedata_tmpp ) );
    to_mux to_mux_EX_rtypedata_n( .choose( func3[3:2]) ,.out(rtypedata_tmpn) ,.a( srl_sra_r ) ,.b({21{n}}) ,.c( slt_r ));
    to_mux to_mux_EX_rtypedata_o( .choose( func3[3:2]) ,.out(rtypedata_tmpo) ,.a( or_r ) ,.b(add_sub_r) ,.c( sltu_r));
    to_mux to_mux_EX_rtypedata_p( .choose( func3[3:2]) ,.out(rtypedata_tmpp) ,.a(and_r ) ,.b(sll_r) ,.c( xor_r));
                    //ooo         pnn       opn     opo      pnp    pno    opp     oop    
    wire [ 41 : 0 ] add_sub_r , srl_sra_r , slt_r , sltu_r , and_r , or_r , xor_r , sll_r ;
    wire [ 41 : 0 ] srl_r , sra_r;

    wire [1:0]is_r_or_sra ,is_r ,is_r_tmp1,is_r_tmp2 , is_r_tmp3 ,is_r_tmp4;
    isn isn_EX_is_r1( .a( opcode[9:8]) ,.out( is_r_tmp1) );
    isp isp_EX_is_r2( .a( opcode[7:6]) ,.out( is_r_tmp2) );
    isp isp_EX_is_r3( .a( opcode[5:4]) ,.out( is_r_tmp3) );
    terand terand_EX_is_r4( .a( is_r_tmp1 ) ,.b(is_r_tmp2) ,.out(is_r_tmp4) );
    terand terand_EX_is_r5( .a( is_r_tmp4) ,.b(is_r_tmp3) ,.out( is_r)) ;
    to_mux to_mux_rs2_data_i_r( .out(rs2_data_i_r) ,.choose( is_r ) ,.a(imm) ,.b( {21{ n}}) ,.c(rs2_data));
    
    //add sub
    wire [ 41 : 0 ] minus_rs2_add_sub;
    wire [ 41 : 0 ] real_rs2_add_sub_r;
    genvar i;
    generate
        for ( i = 0 ; i < 21 ; i = i + 1  ) begin : make_minus_rs2_add_sub
            onot onot_add_sub_r( .a(rs2_data_i_r[i*2 + 1 : i*2] ) ,.out( minus_rs2_add_sub[ i*2 + 1 : i*2 ]) );
        end
    endgenerate
    to_mux to_mux_add_sub_r( .choose(func7[1:0]) ,.out( real_rs2_add_sub_r ),.a( minus_rs2_add_sub ) , .b( rs2_data_i_r )  , .c({21{n}}) ); //n = sub , o = add , p = invalid
    wire [1:0] trash_cout_add_sub_r; 
    wire [ 41 : 0 ] real_rs1_add;
    wire [1:0] rs1_type; //n = pc , o = 0 , p = rs1_data; 
    rsi_check rsi_check_real_rs1( .in(opcode) ,.out(rs1_type));
    to_mux to_mux_real_rs1_add( .a( pre_pc ) , .b( {21{o}} ) , .c( rs1_data ) , .choose(rs1_type) , .out(real_rs1_add));
    baladder_to baladder_to_add_sub_r( .sum(add_sub_r) ,.cout(trash_cout_add_sub_r),.cin(o) ,.a(real_rs1_add) ,.b(real_rs2_add_sub_r));
    
    //sll,srl,sra,sltu,xor,and,or i'll use binary cause using ternary to times 2 or check sign extendtion is kinda slow
    //binary @ai
    wire [31:0] rs1_bi;
    wire [31:0] rs2_bi;
    baltit_to_bit btb_rs1(.in(rs1_data), .out(rs1_bi));
    baltit_to_bit btb_rs2(.in(rs2_data_i_r), .out(rs2_bi));
    wire [4:0] shamt = rs2_bi[4:0];
    wire [31:0] sll_bi;
    wire [31:0] srl_sra_bi;
    wire [31:0] sltu_bi,srl_bi,sra_bi;
    wire [31:0] xor_bi , and_bi , or_bi;
    assign sll_bi = rs1_bi << shamt;
    assign srl_bi = rs1_bi >> shamt;
    assign sra_bi = $signed(rs1_bi) >>> shamt;
    assign sltu_bi = (rs1_bi < rs2_bi) ? 32'd1 : 32'd0;
    assign  and_bi = rs1_bi & rs2_bi ,  or_bi  = rs1_bi | rs2_bi ,  xor_bi = rs1_bi ^ rs2_bi;
    assign srl_sra_bi = (rs2_bi[10] | ~func7[0]) ?sra_bi : srl_bi;
    imm_bit_to_tit imm_bit_to_tit_and_r(  .imm(and_bi), .ter_imm(and_r) );
    imm_bit_to_tit imm_bit_to_tit_or_r (  .imm(or_bi),  .ter_imm(or_r)  );
    imm_bit_to_tit imm_bit_to_tit_xor_r(  .imm(xor_bi), .ter_imm(xor_r) );
    imm_bit_to_tit imm_bit_to_tit_sll_r ( .imm(sll_bi), .ter_imm(sll_r) );
    imm_bit_to_tit imm_bit_to_tit_srl_sra_r(  .imm(srl_sra_bi), .ter_imm(srl_sra_r) );
    // imm_bit_to_tit imm_bit_to_tit_srl_r(  .imm(srl_bi), .ter_imm(srl_r) );
    imm_bit_to_tit imm_bit_to_tit_sltu_r (.imm(sltu_bi),.ter_imm(sltu_r));    
    //end binary @ai
    //slt
    assign slt_r[41 : 2] =  { 20{ o } } ;
    wire [1:0] rs1_rs2_com,rs1_rs2_com_i_r;
    tercom_to tercom_to_rs1_rs2_com( .a( rs1_data )  ,.b( rs2_data ) ,.out(rs1_rs2_com)  );   
    tercom_to tercom_to_EX_slt_r( .a( rs1_data )  ,.b( rs2_data_i_r ) ,.out(rs1_rs2_com_i_r)  );   
    clu clu_EX_slt_r_end( .a( rs1_rs2_com_i_r) ,.out( slt_r[1:0]));
    //load n n n n n 
    wire [ 41 : 0 ] load_data;
    assign load_data = add_sub_r;//同save_data / jalr_data
    
    // n  n  p  n  p
    // o        o  o //我懶著算 可能不用abs
    // oo op pn pp nn
    // o  p  p  o  p 
    // op pn pp op pn
    wire [ 1 : 0 ] read_len_first_tmp1 , read_len_first_tmp2 , read_len_first_tmp3 ,read_len_first_tmp4;
    isn isn_EX_read_len_first( .a( func3[1:0] ) ,.out( read_len_first_tmp1 ) );
    onot onot_EX_read_len_first(  .a(func3[1:0]) ,.out( read_len_first_tmp2 ));
    terany terany_EX_read_len_first( .a( read_len_first_tmp2 )  ,.b( func3[3:2])  ,.out( read_len_first_tmp3  ) );
    teror teror_EX_read_len_first( .a(read_len_first_tmp3)  , .b( read_len_first_tmp1  ) , .out( read_len_first_tmp4)); 
    abs abs_EX_read_len_first( .a( read_len_first_tmp4 )  ,.out( read_wr_len[3:2] ));
    // p  n  p  p  n
    // oo op pn pp nn
    // o  p  o  n  p  //sum 
    // p  n  p  p  n  //pnot
    wire [ 1 : 0 ] read_len_end_tmp ;
    tersum tersum_EX_read_len_end( .a(func3[ 3 : 2 ]) ,.b(func3[ 1 : 0 ]) ,.out( read_len_end_tmp));
    pnot pnot_EX_read_len_end( .a( read_len_end_tmp) ,.out(read_wr_len[1:0]));
    //sign or not
    // oo op pn pp nn
    // o  o  o  p  n //cons
    // p  p  p  n  n //isz
    // n  n  n  p  p //onot
    wire [ 1 : 0 ] read_sign_tmp1,read_sign_tmp2;
    tercons tercons_EX_read_sign(.out( read_sign_tmp1) ,.a( func3[1 : 0] ) ,.b(func3[3 : 2])); 
    isz isz_EX_read_sign( .out( read_sign_tmp2 ) ,.a( read_sign_tmp1));
    onot onot_EX_read_sign( .out(read_sign)  ,.a( read_sign_tmp2));
    //jalr
    //啥都不用做 位址在addi算完了 rd要寫的在上面也寫了 之後配著beq一起寫pc_wr_en就好
    //s type 都寫在上面了 有bug再de
    //b type rs1_rs2_com
    wire [ 1 : 0 ] is_btmp [ 0 : 7 ] ;
    wire [ 1 : 0 ] pc_wr;
    wire [ 1 : 0 ] pc_wr_en_data;
    wire [ 1 : 0 ] pc_wr_en_tmp;
    isp isp_EX_opcode_isb_1( .a(opcode[ 9 : 8 ] ) ,.out( is_btmp[0] ));
    isp isp_EX_opcode_isb_2( .a(opcode[ 7 : 6 ] ) ,.out( is_btmp[1] ));
    isn isn_EX_opcode_isb_3( .a(opcode[ 5 : 4 ] ) ,.out( is_btmp[2] ));
    terand terand_EX_opcode_isb_1( .a(is_btmp[0]) ,.b(is_btmp[1])  ,.out( is_btmp[3]) );
    terand terand_EX_opcode_isb_2( .a(is_btmp[3]) ,.b(is_btmp[2])  ,.out( pc_wr ) );
    terand terand_EX_pc_wr_en_tmp( .a( pc_wr_en_data ) ,.b(pc_wr) ,.out(pc_wr_en_tmp) );
    terand terand_EX_pc_wr_en( .a( pc_wr_en_tmp )  ,.b( EX_change_n) ,.out( pc_wr_en ) );
    

    wire [1:0] beq_data , bne_data , blt_data , bge_data , bltu_data, bgeu_data ;
    isz isz_EX_beq_data(.a( rs1_rs2_com),.out(beq_data) );     //ooo
    onot onot_EX_bne_data( .a( beq_data ) ,.out(bne_data));    //oop
    isp isp_EX_blt_data(.a( rs1_rs2_com),.out(blt_data) );     //opp
    onot onot_EX_bge_data( .a( blt_data ) ,.out( bge_data));   //pnn
    isp isp_EX_bltu_data(.a(sltu_r[1:0]  ),.out(bltu_data) );  //pno
    onot onot_EX_bgeu_data( .a( bltu_data ) ,.out( bgeu_data));//pnp
    wire [1:0] branch ;
    wire [1:0] branch_tmpp ,branch_tmpo ; 
    mux mux_EX_branch_first( .a( {branch_tmpp , branch_tmpo  ,bge_data } ) ,.b( func3[1:0]) ,.out( branch));  
    mux mux_EX_branch_p(  .a( {blt_data , bne_data  ,bgeu_data } ) ,.b( func3[3:2] ) ,.out(branch_tmpp));
    mux mux_EX_branch_o( .a({ n , beq_data,bltu_data } ) ,.b(func3[3:2] ) ,.out( branch_tmpo) );
    
    teror teror_EX_pc_wr_en( .out (pc_wr_en_data)  ,.a( opcode [1:0] ) ,.b(branch));
    //j type 基本跟上面一樣
    wire [ 1 : 0 ] data_tmp [ 0 : 5 ];
    wire [ 1 : 0 ] data_choose;
    isn isn_EX_data_1( .a(opcode[ 9 : 8 ] ) ,.out( data_tmp[0] ));
    isp isp_EX_data_2( .a(opcode[ 5 : 4 ] ) ,.out( data_tmp[1] ));
    isn isn_EX_data_3( .a(opcode[ 3 : 2 ] ) ,.out( data_tmp[2] ));
    isn isn_EX_data_4( .a(opcode[ 1 : 0 ] ) ,.out( data_tmp[3] ));
    terand terand_EX_data_1( .a(data_tmp[0]) ,.b(data_tmp[1])  ,.out( data_tmp[4]) );
    terand terand_EX_data_2( .a(data_tmp[3]) ,.b(data_tmp[2])  ,.out( data_tmp[5]) );
    terand terand_EX_data_3( .a(data_tmp[5]) ,.b(data_tmp[4])  ,.out(   data_choose) );//nxpnn
    to_mux to_mux_EX_data( .a(add_sub_r) ,.b({21{n}}) ,.c(rtypedata) ,.choose(data_choose) ,.out( data)  );
endmodule   
//@ai ai寫的ram
module data_ram (
    input  wire        clk,        // 時脈
    input  wire        we,         // 寫入使能 (Write Enable)
    input  wire [1:0]  size,       // 寫入資料大小控制 (00=byte, 01=half, 10=word)
    input  wire [31:0] addr,       // 記憶體位址
    input  wire [31:0] wdata,      // 要寫入的資料
    output wire [31:0] rdata       // 讀出的資料
    );
    // ===============================
    // 記憶體空間配置: 64KB, word 對齊
    // ===============================
    reg [31:0] mem [0:16383]; // 16K words * 4 bytes = 64KB

    // 將位址對齊到 word (忽略最低兩位)
    wire [13:0] word_addr = addr[15:2];
    wire [1:0]  byte_offset = addr[1:0];

    // ===============================
    // 非同步讀取 (組合邏輯)
    // ===============================
    reg [31:0] rdata_reg;
    always @(*) begin
        case (size)
            2'b00: begin // byte
                case (byte_offset)
                    2'b00: rdata_reg = {{24{mem[word_addr][7]}},  mem[word_addr][7:0]};
                    2'b01: rdata_reg = {{24{mem[word_addr][15]}}, mem[word_addr][15:8]};
                    2'b10: rdata_reg = {{24{mem[word_addr][23]}}, mem[word_addr][23:16]};
                    2'b11: rdata_reg = {{24{mem[word_addr][31]}}, mem[word_addr][31:24]};
                endcase
            end
            2'b01: begin // half word
                if (byte_offset[1] == 1'b0)
                    rdata_reg = {{16{mem[word_addr][15]}}, mem[word_addr][15:0]};
                else
                    rdata_reg = {{16{mem[word_addr][31]}}, mem[word_addr][31:16]};
            end
            2'b10: rdata_reg = mem[word_addr]; // word
            default: rdata_reg = 32'b0;
        endcase
    end
    assign rdata = rdata_reg;

    // ===============================
    // 同步寫入 (posedge clk)
    // ===============================
    always @(posedge clk) begin
        if (we) begin
            case (size)
                2'b00: begin // SB
                    case (byte_offset)
                        2'b00: mem[word_addr][7:0]   <= wdata[7:0];
                        2'b01: mem[word_addr][15:8]  <= wdata[7:0];
                        2'b10: mem[word_addr][23:16] <= wdata[7:0];
                        2'b11: mem[word_addr][31:24] <= wdata[7:0];
                    endcase
                end
                2'b01: begin // SH
                    if (byte_offset[1] == 1'b0)
                        mem[word_addr][15:0]  <= wdata[15:0];
                    else
                        mem[word_addr][31:16] <= wdata[15:0];
                end
                2'b10: begin // SW
                    mem[word_addr] <= wdata;
                end
            endcase
        end
    end
endmodule
//end @ai
module mem ( 
    input [7:0] next_rd,
    input [1:0] pc_wr_en,
    input [1:0] type_mem,  // T ( write rd ) 0( read mem write rd ) 1( write mem )
    input [41:0] data,
    input [41:0] wr_data,
    input [3:0] read_wr_len, //b op , h pn , w pp
    input [ 1:0 ] read_sign, // p = unsign
    input clk,
    output [1:0] pc_wr_en_out,
    output [41:0]data_out, // type o = mem , T = data ;
    output [41:0]pc_wr_addr,//data 
    output [7:0] rd_out//next_rd
    );
    wire [41:0] data_out_tmp;
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;   
    wire [1:0] size ;
    wire [41:0] rdata;
    wire [31:0] wr_data_bi , rdata_bi ,data_bi;
    baltit_to_bit baltit_to_bit_wr_data( .in(wr_data) ,.out(wr_data_bi));
    baltit_to_bit baltit_to_bit_data(.in(data)  ,.out( data_bi));
    mux mux_mem_size( .a({read_wr_len[3] , 5'b00001}) ,.b(read_wr_len[1:0] ) ,.out( size )  );
    data_ram data_ram_mem( .clk( clk ) ,.we( type_mem[1] ) ,.size( size ) , .addr( data_bi ),.wdata( wr_data_bi ) ,.rdata(rdata_bi) );
    to_mux to_mux_mem_data_out_tmp( .out( data_out_tmp) ,.a( data  ) ,.b( rdata  ) ,.c( {21 {n}} ) ,.choose( type_mem ) );
    to_mux to_mux_mem_data_out( .choose( pc_wr_en ) ,.a( data_out_tmp  ) ,.b( {21{n}})  ,.c( wr_data) ,.out(data_out) );
    assign  pc_wr_addr = data,
            rd_out = next_rd;
    isp isp_mem_pc_wr_en_out(.out(pc_wr_en_out) ,.a(pc_wr_en));
    wire [31:0] rdata_unsign_h_bi , rdata_unsign_b_bi ;
    assign  rdata_unsign_h_bi = rdata_bi & { {16{1'b0}} , {16{1'b1}} },
            rdata_unsign_b_bi = rdata_bi & { {24{1'b0}} , { 8{1'b1}} };
    wire [ 41 : 0 ] rdata_u_h , rdata_u_b , rdata_s;
    imm_bit_to_tit ibtt_mem_rdata_s( .imm(rdata_bi) ,.ter_imm(rdata_s));
    imm_bit_to_tit ibtt_mem_rdata_u_h( .imm(rdata_unsign_h_bi) ,.ter_imm(rdata_u_h));
    imm_bit_to_tit ibtt_mem_rdata_u_b( .imm(rdata_unsign_b_bi) ,.ter_imm(rdata_u_b));
    to_mux to_mux_mem_rdata( .a( rdata_s ) ,.b( rdata_u_b ) ,.c( rdata_u_h ) ,.choose(rdata_choose) ,.out( rdata));
    //read_sign & read_wr_len[1:0]
    wire [1:0] rdata_choose;
    terand terand_rdata_choose( .a( read_sign) ,.b(read_wr_len[1:0]) ,.out( rdata_choose));
endmodule
module wb (
    input [ 41:0 ]data,
    input [ 7:0 ] rd,
    output [41:0]data_out,
    output [7:0] rd_out,
    output [1:0] reg_wr_en
    );
    wire [1:0] rd_is_zero;
    wire [1:0] rd_is_zero_tmp1,rd_is_zero_tmp2,rd_is_zero_tmp3,rd_is_zero_tmp4,rd_is_zero_tmp5,rd_is_zero_tmp6;
    isz isz_wb_rd_is_zero1( .a( rd[1:0]) ,.out(  rd_is_zero_tmp1) );
    isz isz_wb_rd_is_zero2( .a( rd[3:2]) ,.out(  rd_is_zero_tmp2) );
    isz isz_wb_rd_is_zero3( .a( rd[5:4]) ,.out(  rd_is_zero_tmp3) );
    isz isz_wb_rd_is_zero4( .a( rd[7:6]) ,.out(  rd_is_zero_tmp4) );
    terand terand_wb_rd_is_zero1( .a( rd_is_zero_tmp1),.b(rd_is_zero_tmp2) ,.out(rd_is_zero_tmp5));
    terand terand_wb_rd_is_zero2( .a( rd_is_zero_tmp5),.b(rd_is_zero_tmp3) ,.out(rd_is_zero_tmp6));
    terand terand_wb_rd_is_zero3( .a( rd_is_zero_tmp6),.b(rd_is_zero_tmp4) ,.out(rd_is_zero));
    onot onot_reg_wr_en( .a( rd_is_zero) ,.out(reg_wr_en));
    assign rd_out = rd , data_out = data;
endmodule
module ctrl(
    input [1:0  ]pc_wr_en,
    input [7:0  ]rd_EX,
    input [7:0  ]rd_mem,
    input [7:0  ]rd_wb,
    input [41:0 ]rd_fowarding_data_mem,
    input [41:0 ]rd_fowarding_data_wb,
    input [7:0  ]rs1,
    input [7:0  ]rs2,
    output [1:0 ]stop/*if stop , id addi x0, x0, 0 */,
    output [1:0 ]EX_change/* EX rd = 0 , type = T */,
    output [41:0]rs1_fowarding,
    output [41:0]rs2_fowarding,
    output [1:0 ]rs1_fow_en,
    output [1:0 ]rs2_fow_en
    );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;  
    genvar i;
    //stop = pc_wr_en || ( rd_EX != 0 //rd_EX_isz_n &&(  rd_EX == rs1 //rs1_rd_eq ) || (  rd_EX == rs2 //rs2_rd_eq ) )
    //EX_change = pc_wr_en
    //rs1_fow_en = (rs1 == rd_mem //rs1_rd_mem_eq || rs1 == rd_wb //rs1_rd_wb_eq) && rs1 != 0;
    //rs2_fow_en = rs2 == rd_mem || rs2 == rd_wb;
    //rs1_fowarding = rd_mem == rs1 ? rd_fowarding_data_mem : rd_fowarding_data_wb;
    //rs2_fowarding = rd_mem == rs2 ? rd_fowarding_data_mem : rd_fowarding_data_wb;

    wire [1:0] rd_EX_isz_tmp[0:5];
    wire [1:0] rd_EX_isz_n;
    isz isz_ctrl_rd_EX1( .a( rd_EX[1:0]) , .out( rd_EX_isz_tmp[0]) );
    isz isz_ctrl_rd_EX2( .a( rd_EX[3:2]) , .out( rd_EX_isz_tmp[1]) );
    isz isz_ctrl_rd_EX3( .a( rd_EX[5:4]) , .out( rd_EX_isz_tmp[2]) );
    isz isz_ctrl_rd_EX4( .a( rd_EX[7:6]) , .out( rd_EX_isz_tmp[3]) );
    terand terand_ctrl_rd_EX1( .a(rd_EX_isz_tmp[0] ) ,.b( rd_EX_isz_tmp[1] ) ,.out( rd_EX_isz_tmp[4] ) );
    terand terand_ctrl_rd_EX2( .a(rd_EX_isz_tmp[4] ) ,.b( rd_EX_isz_tmp[2] ) ,.out( rd_EX_isz_tmp[5] ) );
    ternand ternand_ctrl_rd_EX3( .a(rd_EX_isz_tmp[5] ) ,.b( rd_EX_isz_tmp[3] ) ,.out( rd_EX_isz_n ) );
    wire [7:0] rd_EX_n;
    onot onot_ctrl_rd_EX_n1(.a( rd_EX[7:6] ) ,.out( rd_EX_n[7:6] ));
    onot onot_ctrl_rd_EX_n2(.a( rd_EX[5:4] ) ,.out( rd_EX_n[5:4] ));
    onot onot_ctrl_rd_EX_n3(.a( rd_EX[3:2] ) ,.out( rd_EX_n[3:2] ));
    onot onot_ctrl_rd_EX_n4(.a( rd_EX[1:0] ) ,.out( rd_EX_n[1:0] ));
    //rs1
    wire [ 1:0 ] rs1_rd_any [0 : 3 ];
    wire [ 1:0 ] rs1_rd_eq_tmp [ 0 : 5];
    wire [1:0] rs1_rd_eq;
    terany terany_ctrl_rs1_1( .a( rs1[1:0] ) ,.b( rd_EX_n[1:0] ) ,.out( rs1_rd_any[0] ) );
    terany terany_ctrl_rs1_2( .a( rs1[3:2] ) ,.b( rd_EX_n[3:2] ) ,.out( rs1_rd_any[1] ) );
    terany terany_ctrl_rs1_3( .a( rs1[5:4] ) ,.b( rd_EX_n[5:4] ) ,.out( rs1_rd_any[2] ) );
    terany terany_ctrl_rs1_4( .a( rs1[7:6] ) ,.b( rd_EX_n[7:6] ) ,.out( rs1_rd_any[3] ) );
    isz isz_ctrl_rs1_1( .a(rs1_rd_any[0])  ,.out( rs1_rd_eq_tmp[0] ) );
    isz isz_ctrl_rs1_2( .a(rs1_rd_any[1])  ,.out( rs1_rd_eq_tmp[1] ) );
    isz isz_ctrl_rs1_3( .a(rs1_rd_any[2])  ,.out( rs1_rd_eq_tmp[2] ) );
    isz isz_ctrl_rs1_4( .a(rs1_rd_any[3])  ,.out( rs1_rd_eq_tmp[3] ) );
    terand terand_ctrl_rs1_eq1(.a(rs1_rd_eq_tmp[0]) ,.b( rs1_rd_eq_tmp[1]),.out(rs1_rd_eq_tmp[4])  );
    terand terand_ctrl_rs1_eq2(.a(rs1_rd_eq_tmp[4]) ,.b( rs1_rd_eq_tmp[2]),.out(rs1_rd_eq_tmp[5])  );
    terand terand_ctrl_rs1_eq3(.a(rs1_rd_eq_tmp[5]) ,.b( rs1_rd_eq_tmp[3]),.out(rs1_rd_eq)  );
    //rs2
    wire [ 1:0 ] rs2_rd_any [0 : 3 ];
    wire [ 1:0 ] rs2_rd_eq_tmp [ 0 : 5];
    wire [1:0] rs2_rd_eq;
    terany terany_ctrl_rs2_1( .a( rs2[1:0] ) ,.b( rd_EX_n[1:0] ) ,.out( rs2_rd_any[0] ) );
    terany terany_ctrl_rs2_2( .a( rs2[3:2] ) ,.b( rd_EX_n[3:2] ) ,.out( rs2_rd_any[1] ) );
    terany terany_ctrl_rs2_3( .a( rs2[5:4] ) ,.b( rd_EX_n[5:4] ) ,.out( rs2_rd_any[2] ) );
    terany terany_ctrl_rs2_4( .a( rs2[7:6] ) ,.b( rd_EX_n[7:6] ) ,.out( rs2_rd_any[3] ) );
    isz isz_ctrl_rs2_1( .a(rs2_rd_any[0])  ,.out( rs2_rd_eq_tmp[0] ) );
    isz isz_ctrl_rs2_2( .a(rs2_rd_any[1])  ,.out( rs2_rd_eq_tmp[1] ) );
    isz isz_ctrl_rs2_3( .a(rs2_rd_any[2])  ,.out( rs2_rd_eq_tmp[2] ) );
    isz isz_ctrl_rs2_4( .a(rs2_rd_any[3])  ,.out( rs2_rd_eq_tmp[3] ) );
    terand terand_ctrl_rs2_eq1(.a(rs2_rd_eq_tmp[0]) ,.b( rs2_rd_eq_tmp[1]),.out(rs2_rd_eq_tmp[4])  );
    terand terand_ctrl_rs2_eq2(.a(rs2_rd_eq_tmp[4]) ,.b( rs2_rd_eq_tmp[2]),.out(rs2_rd_eq_tmp[5])  );
    terand terand_ctrl_rs2_eq3(.a(rs2_rd_eq_tmp[5]) ,.b( rs2_rd_eq_tmp[3]),.out(rs2_rd_eq)  );
    wire [1:0]stop_tmp1 , stop_tmp2;
    teror teror_ctrl_stop1( .a(rs1_rd_eq ) ,.b( rs2_rd_eq)  ,.out( stop_tmp1) );
    terand terand_ctrl_stop2( .a(rd_EX_isz_n ) ,.b( stop_tmp1)  ,.out( stop_tmp2) );
    teror teror_ctrl_stop3( .a( stop_tmp2) ,.b( pc_wr_en ) ,.out( stop )   );

    assign EX_change = pc_wr_en;

    wire [1:0] rs1_isz_n;
    wire [1:0] rs1_isz_tmp[0:5];
    isz isz_ctrl_isz_tmp0( .a( rs1[1:0]) ,.out( rs1_isz_tmp[0]));
    isz isz_ctrl_isz_tmp1( .a( rs1[3:2]) ,.out( rs1_isz_tmp[1]));
    isz isz_ctrl_isz_tmp2( .a( rs1[5:4]) ,.out( rs1_isz_tmp[2]));
    isz isz_ctrl_isz_tmp3( .a( rs1[7:6]) ,.out( rs1_isz_tmp[3]));
    terand terand_ctrl_isz_tmp4( .a( rs1_isz_tmp[0]) ,.b( rs1_isz_tmp[1]) ,.out( rs1_isz_tmp[4]) );
    terand terand_ctrl_isz_tmp5( .a( rs1_isz_tmp[4]) ,.b( rs1_isz_tmp[2]) ,.out( rs1_isz_tmp[5]) );
    ternand ternand_ctrl_isz_tmp6( .a( rs1_isz_tmp[5]) ,.b( rs1_isz_tmp[3]) ,.out( rs1_isz_n) );
    wire [7:0] rs1_n;
    onot onot_ctrl_rs1_n1(.a( rs1[7:6] ) ,.out( rs1_n[7:6] ));
    onot onot_ctrl_rs1_n2(.a( rs1[5:4] ) ,.out( rs1_n[5:4] ));
    onot onot_ctrl_rs1_n3(.a( rs1[3:2] ) ,.out( rs1_n[3:2] ));
    onot onot_ctrl_rs1_n4(.a( rs1[1:0] ) ,.out( rs1_n[1:0] ));
    //rd_mem
    wire [ 1:0 ] rs1_rd_mem_any [0 : 3 ];
    wire [ 1:0 ] rs1_rd_mem_eq_tmp [ 0 :5 ];
    wire [1:0] rs1_rd_mem_eq;
    terany terany_ctrl_rs1_mem_1( .a( rs1_n[1:0] ) ,.b( rd_mem[1:0] ) ,.out( rs1_rd_mem_any[0] ) );
    terany terany_ctrl_rs1_mem_2( .a( rs1_n[3:2] ) ,.b( rd_mem[3:2] ) ,.out( rs1_rd_mem_any[1] ) );
    terany terany_ctrl_rs1_mem_3( .a( rs1_n[5:4] ) ,.b( rd_mem[5:4] ) ,.out( rs1_rd_mem_any[2] ) );
    terany terany_ctrl_rs1_mem_4( .a( rs1_n[7:6] ) ,.b( rd_mem[7:6] ) ,.out( rs1_rd_mem_any[3] ) );
    isz isz_ctrl_rs1_mem_1( .a(rs1_rd_mem_any[0])  ,.out( rs1_rd_mem_eq_tmp[0] ) );
    isz isz_ctrl_rs1_mem_2( .a(rs1_rd_mem_any[1])  ,.out( rs1_rd_mem_eq_tmp[1] ) );
    isz isz_ctrl_rs1_mem_3( .a(rs1_rd_mem_any[2])  ,.out( rs1_rd_mem_eq_tmp[2] ) );
    isz isz_ctrl_rs1_mem_4( .a(rs1_rd_mem_any[3])  ,.out( rs1_rd_mem_eq_tmp[3] ) );
    terand terand_ctrl_rs1_mem_eq1(.a(rs1_rd_mem_eq_tmp[0]) ,.b( rs1_rd_mem_eq_tmp[1]),.out(rs1_rd_mem_eq_tmp[4])  );
    terand terand_ctrl_rs1_mem_eq2(.a(rs1_rd_mem_eq_tmp[4]) ,.b( rs1_rd_mem_eq_tmp[2]),.out(rs1_rd_mem_eq_tmp[5])  );
    terand terand_ctrl_rs1_mem_eq3(.a(rs1_rd_mem_eq_tmp[5]) ,.b( rs1_rd_mem_eq_tmp[3]),.out(rs1_rd_mem_eq)  );
    //rd_wb
    wire [ 1:0 ] rs1_rd_wb_any [0 : 3 ];
    wire [ 1:0 ] rs1_rd_wb_eq_tmp [ 0 : 5];
    wire [1:0] rs1_rd_wb_eq ,rs1_fow_en_tmp;
    terany terany_ctrl_rs1_wb_1( .a( rs1_n[1:0] ) ,.b( rd_wb[1:0] ) ,.out( rs1_rd_wb_any[0] ) );
    terany terany_ctrl_rs1_wb_2( .a( rs1_n[3:2] ) ,.b( rd_wb[3:2] ) ,.out( rs1_rd_wb_any[1] ) );
    terany terany_ctrl_rs1_wb_3( .a( rs1_n[5:4] ) ,.b( rd_wb[5:4] ) ,.out( rs1_rd_wb_any[2] ) );
    terany terany_ctrl_rs1_wb_4( .a( rs1_n[7:6] ) ,.b( rd_wb[7:6] ) ,.out( rs1_rd_wb_any[3] ) );
    isz isz_ctrl_rs1_wb_1( .a(rs1_rd_wb_any[0])  ,.out( rs1_rd_wb_eq_tmp[0] ) );
    isz isz_ctrl_rs1_wb_2( .a(rs1_rd_wb_any[1])  ,.out( rs1_rd_wb_eq_tmp[1] ) );
    isz isz_ctrl_rs1_wb_3( .a(rs1_rd_wb_any[2])  ,.out( rs1_rd_wb_eq_tmp[2] ) );
    isz isz_ctrl_rs1_wb_4( .a(rs1_rd_wb_any[3])  ,.out( rs1_rd_wb_eq_tmp[3] ) );
    terand terand_ctrl_rs1_wb_eq1(.a(rs1_rd_wb_eq_tmp[0]) ,.b( rs1_rd_wb_eq_tmp[1]),.out(rs1_rd_wb_eq_tmp[4])  );
    terand terand_ctrl_rs1_wb_eq2(.a(rs1_rd_wb_eq_tmp[4]) ,.b( rs1_rd_wb_eq_tmp[2]),.out(rs1_rd_wb_eq_tmp[5])  );
    terand terand_ctrl_rs1_wb_eq3(.a(rs1_rd_wb_eq_tmp[5]) ,.b( rs1_rd_wb_eq_tmp[3]),.out(rs1_rd_wb_eq)  );
    teror teror_ctrl_rs1_fow_en_tmp(.a( rs1_rd_wb_eq),.b( rs1_rd_mem_eq)  , .out(rs1_fow_en_tmp) ); 
    terand terand_ctrl_rs1_fow_en(.a( rs1_isz_n ),.b( rs1_fow_en_tmp)  , .out(rs1_fow_en) ); 
    
    wire [1:0] rs2_isz_n;
    wire [1:0] rs2_isz_tmp[0:5];
    isz isz_ctrl_rs2_isz_tmp0( .a( rs2[1:0]) ,.out( rs2_isz_tmp[0]));
    isz isz_ctrl_rs2_isz_tmp1( .a( rs2[3:2]) ,.out( rs2_isz_tmp[1]));
    isz isz_ctrl_rs2_isz_tmp2( .a( rs2[5:4]) ,.out( rs2_isz_tmp[2]));
    isz isz_ctrl_rs2_isz_tmp3( .a( rs2[7:6]) ,.out( rs2_isz_tmp[3]));
    terand terand_ctrl_rs2_isz_tmp4( .a( rs2_isz_tmp[0]) ,.b( rs2_isz_tmp[1]) ,.out( rs2_isz_tmp[4]) );
    terand terand_ctrl_rs2_isz_tmp5( .a( rs2_isz_tmp[4]) ,.b( rs2_isz_tmp[2]) ,.out( rs2_isz_tmp[5]) );
    ternand ternand_ctrl_rs2_isz_tmp6( .a( rs2_isz_tmp[5]) ,.b( rs2_isz_tmp[3]) ,.out( rs2_isz_n) );
    wire [7:0] rs2_n;
    onot onot_ctrl_rs2_n1(.a( rs2[7:6] ) ,.out( rs2_n[7:6] ));
    onot onot_ctrl_rs2_n2(.a( rs2[5:4] ) ,.out( rs2_n[5:4] ));
    onot onot_ctrl_rs2_n3(.a( rs2[3:2] ) ,.out( rs2_n[3:2] ));
    onot onot_ctrl_rs2_n4(.a( rs2[1:0] ) ,.out( rs2_n[1:0] ));
    //rd_mem
    wire [ 1:0 ] rs2_rd_mem_any [0 : 3 ];
    wire [ 1:0 ] rs2_rd_mem_eq_tmp [ 0 : 5];
    wire [1:0] rs2_rd_mem_eq;
    terany terany_ctrl_rs2_mem_1( .a( rs2_n[1:0] ) ,.b( rd_mem[1:0] ) ,.out( rs2_rd_mem_any[0] ) );
    terany terany_ctrl_rs2_mem_2( .a( rs2_n[3:2] ) ,.b( rd_mem[3:2] ) ,.out( rs2_rd_mem_any[1] ) );
    terany terany_ctrl_rs2_mem_3( .a( rs2_n[5:4] ) ,.b( rd_mem[5:4] ) ,.out( rs2_rd_mem_any[2] ) );
    terany terany_ctrl_rs2_mem_4( .a( rs2_n[7:6] ) ,.b( rd_mem[7:6] ) ,.out( rs2_rd_mem_any[3] ) );
    isz isz_ctrl_rs2_mem_1( .a(rs2_rd_mem_any[0])  ,.out( rs2_rd_mem_eq_tmp[0] ) );
    isz isz_ctrl_rs2_mem_2( .a(rs2_rd_mem_any[1])  ,.out( rs2_rd_mem_eq_tmp[1] ) );
    isz isz_ctrl_rs2_mem_3( .a(rs2_rd_mem_any[2])  ,.out( rs2_rd_mem_eq_tmp[2] ) );
    isz isz_ctrl_rs2_mem_4( .a(rs2_rd_mem_any[3])  ,.out( rs2_rd_mem_eq_tmp[3] ) );
    terand terand_ctrl_rs2_mem_eq1(.a(rs2_rd_mem_eq_tmp[0]) ,.b( rs2_rd_mem_eq_tmp[1]),.out(rs2_rd_mem_eq_tmp[4])  );
    terand terand_ctrl_rs2_mem_eq2(.a(rs2_rd_mem_eq_tmp[4]) ,.b( rs2_rd_mem_eq_tmp[2]),.out(rs2_rd_mem_eq_tmp[5])  );
    terand terand_ctrl_rs2_mem_eq3(.a(rs2_rd_mem_eq_tmp[5]) ,.b( rs2_rd_mem_eq_tmp[3]),.out(rs2_rd_mem_eq)  );
    //rd_wb
    wire [ 1:0 ] rs2_rd_wb_any [0 : 3 ];
    wire [ 1:0 ] rs2_rd_wb_eq_tmp [ 0 : 5 ];
    wire [1:0] rs2_rd_wb_eq , rs2_fow_en_tmp;
    terany terany_ctrl_rs2_wb_1( .a( rs2_n[1:0] ) ,.b( rd_wb[1:0] ) ,.out( rs2_rd_wb_any[0] ) );
    terany terany_ctrl_rs2_wb_2( .a( rs2_n[3:2] ) ,.b( rd_wb[3:2] ) ,.out( rs2_rd_wb_any[1] ) );
    terany terany_ctrl_rs2_wb_3( .a( rs2_n[5:4] ) ,.b( rd_wb[5:4] ) ,.out( rs2_rd_wb_any[2] ) );
    terany terany_ctrl_rs2_wb_4( .a( rs2_n[7:6] ) ,.b( rd_wb[7:6] ) ,.out( rs2_rd_wb_any[3] ) );
    isz isz_ctrl_rs2_wb_1( .a(rs2_rd_wb_any[0])  ,.out( rs2_rd_wb_eq_tmp[0] ) );
    isz isz_ctrl_rs2_wb_2( .a(rs2_rd_wb_any[1])  ,.out( rs2_rd_wb_eq_tmp[1] ) );
    isz isz_ctrl_rs2_wb_3( .a(rs2_rd_wb_any[2])  ,.out( rs2_rd_wb_eq_tmp[2] ) );
    isz isz_ctrl_rs2_wb_4( .a(rs2_rd_wb_any[3])  ,.out( rs2_rd_wb_eq_tmp[3] ) );
    terand terand_ctrl_rs2_wb_eq1(.a(rs2_rd_wb_eq_tmp[0]) ,.b( rs2_rd_wb_eq_tmp[1]),.out(rs2_rd_wb_eq_tmp[4])  );
    terand terand_ctrl_rs2_wb_eq2(.a(rs2_rd_wb_eq_tmp[4]) ,.b( rs2_rd_wb_eq_tmp[2]),.out(rs2_rd_wb_eq_tmp[5])  );
    terand terand_ctrl_rs2_wb_eq3(.a(rs2_rd_wb_eq_tmp[5]) ,.b( rs2_rd_wb_eq_tmp[3]),.out(rs2_rd_wb_eq)  );
    teror teror_ctrl_rs2_fow_en_tmp( .a( rs2_rd_wb_eq),.b( rs2_rd_mem_eq)  , .out(rs2_fow_en_tmp) );
    terand terand_ctrl_rs2_fow_en( .a(  rs2_isz_n),.b( rs2_fow_en_tmp)  , .out(rs2_fow_en) );

    
    wire [41:0] rs1_fow_wb, rs1_fow_mem , rs2_fow_wb , rs2_fow_mem ; 
    generate
        for( i = 0 ; i < 21 ; i = i + 1) begin : ctrl_rs1_rs2_fowarding_array
            terand terand_ctrl_rs1_fow_wb( .out(rs1_fow_wb[i*2+1 : i*2]) ,.a( rd_fowarding_data_wb[ i*2+1 : i*2 ]) ,.b( rs1_rd_wb_eq ) );
            terand terand_ctrl_rs1_fow_mem( .out(rs1_fow_mem[i*2 +1 : i*2]) ,.a( rd_fowarding_data_mem[ i*2+1 : i*2 ]) ,.b( rs1_rd_mem_eq ) );
            teror teror_ctrl_rs1_fowarding( .a( rs1_fow_wb[i*2+1:i*2]) ,.b( rs1_fow_mem[i*2+1:i*2] ) ,.out( rs1_fowarding[i*2+1:i*2] ) );

            terand terand_ctrl_rs2_fow_wb( .out(rs2_fow_wb[i*2+1 : i*2]) ,.a( rd_fowarding_data_wb[ i*2+1 : i*2 ]) ,.b( rs2_rd_wb_eq ) );
            terand terand_ctrl_rs2_fow_mem( .out(rs2_fow_mem[i*2 +1 : i*2]) ,.a( rd_fowarding_data_mem[ i*2+1 : i*2 ]) ,.b( rs2_rd_mem_eq ) );
            teror teror_ctrl_rs2_fowarding( .a( rs2_fow_wb[i*2+1:i*2]) ,.b( rs2_fow_mem[i*2+1:i*2] ) ,.out( rs2_fowarding[i*2+1:i*2] ) );
        end
    endgenerate
endmodule
/* verilator lint_on DECLFILENAME */
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
/* verilator lint_on UNOPTFLAT */
/* verilator lint_on WIDTHEXPAND */
/* verilator lint_on WIDTHTRUNC */
/* verilator lint_on CASEINCOMPLETE */

