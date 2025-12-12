/* verilator lint_off DECLFILENAME */
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off UNOPTFLAT */
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
/* verilator lint_off CASEINCOMPLETE */

//假設https://louis-dr.github.io/ternlogic.html 都已經實做出來

module terand(input [1:0]a,input [1:0]b,output reg [1:0]out);
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    always @(*) begin
        case (a)
            n:out = n;
            o:begin
                case(b)
                    p:out = o;
                    o:out= o;
                    n:out = n;
                    default :out = o;
                endcase
            end
            p:begin
                case(b)
                    p:out = p;
                    o: out = o;
                    n: out= n;
                    default :out = p;
                endcase
            end
            default:out = o;
        endcase
    end
endmodule
module teror(input [1:0]a,input [1:0]b,output reg [1:0]out);
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    always @(*) begin
        case (a)
            p:out = p;
            o:begin
                case(b)
                    p:out = p;
                    o:out= o;
                    n:out = o;
                    default :out = o;
                endcase
            end
            n:begin
                case(b)
                    p:out = p;
                    o: out = o;
                    n: out= n;
                    default :out = n;
                endcase
            end
            default:out = o;
        endcase
    end
endmodule
module onot( input [1:0] a , output [1:0] out );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out = (a==o)?o:((a==p)?n:p);
endmodule
module pnot( input [1:0] a, output [1:0]out );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out = (a == p)?n : p;
endmodule
module nnot( input [1:0] a, output [1:0]out );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out = (a == n)? p: n;
endmodule
module abs(input [1:0] a, output [1:0]out);
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out= ( a == o )?o : p;
endmodule
module clu(input [1:0] a, output [1:0]out);
    parameter  p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out = (a == p )? p: o;
endmodule
module cld(input [1:0] a, output [1:0]out);
    parameter  p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out =  ( a== n )? n: o; 
endmodule
module inc(input [1:0] a,output [1:0] out);
    parameter  p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out = ( a== n)? o :  p; 
endmodule
module dec(input [1:0] a,output [1:0] out);
    parameter  p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out = (a == p  )? o: n;
endmodule
module rtu( input [1:0] a,output reg [1:0] out);
    parameter  p = 2'b10 , n = 2'b00 , o = 2'b01;
    always @(*) begin
    case(a)
        n:out = o;
        o:out = p;
        p:out = n;
        default : out = o;
    endcase
    end
endmodule
module rtd(  input [1:0] a,output reg [1:0] out );
    parameter  p = 2'b10 , n = 2'b00 , o = 2'b01;
    always @(*)begin
    case(a)
        n:out = p;
        o:out = n;
        p:out = o;
        default : out = o;
    endcase
    end
endmodule
module isz (input [1:0] a,output [1:0] out);
    parameter  p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out = (a ==o)?p : n; 
endmodule
module isp (input [1:0] a,output [1:0] out);
    parameter  p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out = (a ==p)?p : n; 
endmodule
module isn (input [1:0] a,output [1:0] out);
    parameter  p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out = (a == n)?p : n; 
endmodule
module ternand(input [1:0]a, input [1:0]b, output [1:0]out);
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [1:0] r;
    terand terand_ternand(.a(a) , .b(b) , .out(r));
    onot onot_nand(.a(r) , .out(out));
endmodule
module ternor(input [1:0]a, input [1:0]b, output [1:0]out);
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [1:0] r;
    teror teror_ternor(.a(a),.b(b),.out(r));
    onot onot_nor(.a(r) , .out(out));
endmodule
module ternmul (input [1:0]a, input [1:0]b ,output [1:0]out);
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [1:0] wire1,wire2,wire3;
    ternand ternand_ternmul1(.a(a) , .b(b) , .out( wire1) );
    ternand ternand_ternmul2(.a(a) , .b(wire1) , .out( wire2) );
    ternand ternand_ternmul3(.a(b) , .b(wire1) , .out( wire3) );
    ternand ternand_ternmul4(.a(wire2) , .b(wire3) , .out(out));
    /*
    truth table
    | a | b | out |
    | n | n |  n  |
    | n | o |  o  |
    | n | p |  p  |
    | o | n |  o  |
    | o | o |  o  |
    | o | p |  o  |
    | p | n |  p  |
    | p | o |  o  |
    | p | p |  n  |
    */
endmodule
module termul(input [1:0] a, input [1:0] b ,output [1:0]out );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [1:0]wire1;
    ternmul ternmul_terxnor(.a(a) , .b(b) , .out(wire1));
    onot onot_xnor(.a(wire1) , .out(out));
endmodule
module tercons( input [1:0] a, input [1:0] b ,output [1:0]out );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    assign out = (a == b)? a: o; 
endmodule
module terncons( input [1:0] a, input [1:0] b ,output [1:0]out );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [1:0]wire1;
    tercons tercons_terncons(.a(a) ,.b(b) ,.out(wire1));
    onot onot_terncons (.a(wire1) , .out(out));
endmodule
module terany(input [1:0] a, input [1:0] b ,output reg [1:0]out );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    always @(*) begin
        case(a)
            p:out = (b==n)?o:p;
            o:out =b;
            n:out = (b==p)?o:n;
            default: out = 2'b0;
        endcase
    end
endmodule
module ternany (input [1:0] a, input [1:0] b ,output [1:0]out );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [1:0]wire1;
    terany terany_ternany(.a(a), .b(b) ,.out(wire1));
    onot onot_ternany(.a(wire1 ) , .out(out));
endmodule
module tersum( input [1:0] a, input [1:0] b ,output reg [1:0]out );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    always @(*) begin
        case(a)
            p:out = (b== p)?n:b+2'b1;
            o:out = b;
            n:out = (b==n)?p:b-2'b1;
            default : out = 2'b0;
        endcase    
    end    
endmodule
module ternsum( input [1:0] a, input [1:0] b ,output  [1:0]out );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [1:0]wire1;
    tersum tersum_ternsum( .a(a) , .b(b) , .out(wire1)) ;
    onot onot_ternsum(.a(wire1 ) , .out(out));
endmodule

//man make 

module decoder( input [1:0] a, output [1:0] po, ze ,ne);
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    isz isz_decoder (.a(a) , .out(ze));
    isn isn_decoder (.a(a) , .out(ne));
    isp isp_decoder (.a(a) , .out(po));
endmodule
module mux( input [5:0] a, input [1:0]b , output [1:0]out);//pon ,shit why did i use pon
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [1:0] po ,ze ,ne; 
    decoder decoder_mux( .a(b) ,.po( po ) , .ze( ze ),.ne( ne ) );
    wire [1:0 ]wire1,wire2,wire3,wire4 ;  
    terand terand_mux1( .a(a[5:4])  ,.b(po),.out(wire1));
    terand terand_mux2( .a(a[3:2])  ,.b(ze),.out(wire2)); 
    terand terand_mux3( .a(a[1:0])  ,.b(ne),.out(wire3));
    teror teror_mux1(.a(wire1) , .b(wire2) ,.out(wire4));
    teror teror_mux2(.a(wire4) , .b(wire3) ,.out(out)  );
endmodule
module tersrlatch(input[1:0] s,input[1:0] r,input clk , input[1:0] rst_n,output [1:0]q);
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    wire [1:0]wire1,wire2,qbar,wire3,wire4,wire5,wire6 , rst_o;
    assign q = wire4;
    terand terand_tersrlatch1( .a(s) , .b({clk,1'b0}) , .out(wire1) );
    terand terand_tersrlatch2( .a(r) , .b({clk,1'b0}) , .out(wire2) );
    ternor ternor_tersrlatch1( .a(wire1) , .b(wire4)  , .out(wire5) );
    ternor ternor_tersrlatch2( .a(wire2) , .b(qbar ) , .out(wire3 ) );
    teror teror_tersrlath_rst_o( .a(o) ,.b( rst_n) ,.out( rst_o) );
    termul termul_tersrlatch3(.a(wire3) , .b(rst_o) ,.out(wire4));
    termul termul_tersrlatch(.a(wire5) , .b(rst_o) ,.out(qbar));
endmodule
module terdlatch (input [1:0]d,input clk,input[1:0]rst_n, output [1:0]q );
    wire [1:0]wire1;
    onot onot_terdlatch( .a(d) , .out(wire1));
    tersrlatch tersrlatch_terterdlatch( .rst_n(rst_n),.s(d ) , .r(wire1) ,.clk(clk ) ,.q(q)) ;   
endmodule
module terdff(input[1:0] d, input clk,input[1:0] rst_n, output [1:0]q);
    wire [1:0]wire1;
    terdlatch terdlatch_terdff1(.rst_n(rst_n), .d(d) , .clk(~clk) , .q(wire1));
    terdlatch terdlatch_terdff2(.d(wire1),.rst_n(rst_n) , .clk(clk) , .q(q));
endmodule
module terdff_to (input[41:0] d, input clk,input[1:0] rst_n, output [41:0]q);
    genvar i;
    generate 
        for( i = 0 ; i < 21 ; i=i +1)begin : terdff_array
            terdff terdff_terdff_to(  .d(d[i*2  +: 2 ])  ,.clk(clk ) ,.rst_n(rst_n) ,.q(q[i*2+:2])  );            
        end
    endgenerate
endmodule
module baltit_to_bit (input  [41:0] in,output reg [31:0] out);//[to_do]
    parameter p = 2'b10,n = 2'b00, o = 2'b01;
    
    integer i,value; 
    always @(*) begin
        value = 0;
        for (i = 0; i < 21; i = i + 1) begin
            case (in[i*2 +: 2])
                n: value = value - (3 ** i);
                p: value = value + (3 ** i); 
                default: value = value;   
            endcase
        end
        out = value[31:0]; 
    end
endmodule
module baladder(input [1:0]a, input [1:0]b, input [1:0] cin,output [1:0]sum , output [1:0]cout );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    //cout
    wire [1:0] wire1 , wire2 , wire3,wire4 ,wire5,mul1,mul2,mul3 ,wire6,wire7 ,wire8;
    teror teror_baladder1(.a(a) ,.b(b) ,.out( wire1));
    terand terand_baladder1(.a(a) ,.b(b) ,.out( wire2));
    isz isz_baladder1(.a(wire2) , .out(wire3));
    mux mux_baladder1(.a( {wire1 , o , wire2}) , .b(wire3) ,.out(wire4));
    termul termul_baladder1( .a(a) , .b(b) ,.out(mul1));
    termul termul_baladder2( .a(cin) , .b(b) ,.out(mul2));
    termul termul_baladder3( .a(cin) , .b(a) ,.out(mul3));
    mux mux_baladder2( .a({o,wire4 ,o}) ,.b(mul1) ,.out(wire5));
    mux mux_baladder3( .a({wire4, o ,o} ),.b(mul1) ,.out(wire6));
    mux mux_baladder4(.a({wire6 ,wire5 ,o} ) ,.b(mul2) ,.out(wire7));
    mux mux_baladder5(.a({wire5 ,wire6 ,o} ) ,.b(mul2) ,.out(wire8)); 
    mux mux_baladder6(.a({wire7 , wire8 , o } ) ,.b(mul3 ) ,.out(cout));
    //sum
    wire [1:0] wire9;
    tersum tersum_adder1( .a(a) , .b(b) ,.out(wire9));
    tersum tersum_adder2(.a(cin ) ,.b(wire9) ,.out(sum));
endmodule
module bit_to_tit_force #( parameter is_ter_make = 0 , parameter OUT_WIDTH = 2 )(  input  bi , output [ OUT_WIDTH-1 : 0 ] ter );// para 0 = not make use bin, para 1 = use + 0 , para 2 = use + -   
    generate 
        if ( is_ter_make == 0  )
        begin : bin_sim
            assign ter = { bi , 1'b0  };       
        end
        else if( is_ter_make == 1 ) begin : ter_type_1 //1 , 0 = + , 0 
            isp isp_bit_to_tit( .a( bi ) ,.out( ter ) );
        end
        else begin :ter_type2 // 1 , 0 = + , -
            assign ter = bi ;
        end
    endgenerate
endmodule
module imm_bit_to_tit( input [31:0] imm , output [41:0] ter_imm); //[to_do]
    wire [31:0] real_num;
    assign real_num = (imm[31] ==1'b1)?( ~imm  +1):imm; 
    integer num ,i ;
    reg [41:0] nonbaltit ;wire [41:0] ter_imm_ori , ter_imm_onot; 
    always @(*) begin
        i = 0;
        for(num = real_num  ; num > 0  ; num/=3)
        begin
            nonbaltit[ 2*i +:2] = num %3;
            i++;
        end
        for(i=i ; i < 21 ; i = i + 1 )
            nonbaltit[2*i +: 2] =2'b00;
     end
    nonbaltit_to_baltit nonbaltit_to_baltit_imm_bit_to_tit( .in(nonbaltit) ,.out(ter_imm_ori));
    genvar ii ;
    generate
        for(ii = 0 ; ii < 21 ; ii= ii+1)begin : onot_array
            onot onot_imm_bit_to_tit( .a(ter_imm_ori[ii*2 +1 : ii*2 ]) ,.out(ter_imm_onot[ ii*2+1 : ii*2 ]));
        end
    endgenerate
    assign ter_imm = (imm[31] ==1'b1)? ter_imm_onot : ter_imm_ori;
endmodule
module nonbaltit_to_baltit(input [41:0] in , output [41:0] out ) ; //[to_do]
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    reg [41:0] wire1;
    reg [41:0] cin ; 
    wire [1:0] cout ; 
    integer l;
    always @(*) begin
        for(l = 0 ; l < 21;l = l+1)
            begin
                wire1 [l*2 +:2]  = ( in[l*2 +:2 ] == p )? n:  in[ l*2 +: 2]+1;
                cin [l*2 +:2 ] = ( in[l*2 +:2 ] == p )? p : o ; 
            end
    end
    baladder_to baladder_to_nonbaltit_to_baltit(.a(wire1)  ,.b( {cin[39:0],2'b01} ) ,.cin(2'b01) ,.sum(out) ,.cout(cout));
endmodule
module baladder_to ( input [41:0] a,input [41:0] b, input [1:0] cin , output [41:0] sum ,output [1:0] cout );
    genvar i;
    wire [41:0] coutt ;
    baladder baladder_to1 (
                .a(a[1 : 0]) ,
                .b(b[1 :0] ) ,
                .cin(cin) ,
                .sum(sum[1:0]),
                .cout(coutt[1:0])
                );
    generate
        for( i = 1  ; i< 21 ;i = i+1) begin:baladder_to
            baladder baladder_to (
                    .a(a[i*2 +1 : i*2]) ,
                    .b(b[i*2 +1 : i*2] ) ,
                    .cin(coutt [i *2 -1: i*2-2]) ,
                    .sum(sum[i*2+1:i*2]),
                    .cout(coutt[i*2+1 : i*2])
                    );
        end
    endgenerate
    assign cout = coutt[41:40];
endmodule
module nonbaladder(input [1:0]a, input [1:0]b, input [1:0] cin,output [1:0]sum , output [1:0]cout );
    parameter p = 2'b10 , n = 2'b00 , o = 2'b01;
    //cout
    wire [1:0] wire1,wire2,wire3,wire4,wire5,wire6,wire7,wire8,wire9,wire10,wire11;
    terand terand_adder1(.a(cin) , .b(b) ,.out(wire1));
    terand terand_adder2(.a(a) , .b(wire1) ,.out(wire2));
    teror teror_adder1(.a(a) , .b(b) , .out(wire3));
    teror teror_adder2(.a(wire3) , .b(cin) , .out(wire4));
    isp isp_adder1(.a(wire4) ,.out(wire5));
    ternmul ternmul_adder1(.a(a) , .b(b) , .out(wire6));
    ternmul ternmul_adder2(.a(wire6) , .b(cin ) , .out(wire7));
    isp isp_adder2(.a(wire7) , .out(wire8));
    onot onot_adder1(.a(wire8) , .out(wire9));
    terand terand_adder3(.a(wire9) , .b(wire5) , .out(wire10));
    mux mux_adder1(.a(6'b010000) ,.b(wire10) ,.out(wire11));
    mux mux_adder2(.a( {4'b0110 , wire11} ) , .b(wire2), .out(cout) );
    //sum
    wire [1:0] wire12,wire13,wire14,wire15,wire16;
    rtu rtu_adder1( .a(a) , .out(wire12));
    rtu rtu_adder2( .a(wire12) , .out(wire13));
    mux mux_adder3(.a({wire13,wire12,a})  , .b(b) , .out(wire14));
    rtu rtu_adder3( .a(wire14) , .out(wire15));
    rtu rtu_adder4( .a(wire15) , .out(wire16));
    mux mux_adder4(.a({wire16,wire15,wire14})  , .b(cin ) , .out(sum));
endmodule
module to_mux(   input [41 :0] a , input [41 :0 ] b , input [ 41 :0 ] c  , input [1:0] choose , output [41:0]out ); //left-1 mid0 right1
    wire [1:0] po ,ze ,ne; 
    decoder decoder_to_mux( .a(choose) ,.po(po ) ,.ze( ze ) ,.ne(ne));
    genvar l;
    wire[41:0] wire1 ,wire2 ,wire3,wire4;
    generate
        for( l = 0 ; l< 21 ; l = l+1 )begin : and_or_array
            terand terand_to_mux_po(
                .a(c[l*2 +:2]),
                .b(po), 
                .out(wire1[l*2 +:2])    
                );
            terand terand_to_mux_mid(
                .a(b[l*2 +:2]),
                .b(ze),
                .out(wire2[l*2 +:2] )   
                );
            terand terand_to_mux_ne(
                .a(a[l*2 +:2] ),
                .b(ne), 
                .out(wire3[l*2 +:2])    
                );
            teror teror_to_mux_1(
                .a(wire1[l*2 +:2]),
                .b(wire2[l*2 +:2]),
                .out(wire4[l*2 +:2])
                );
            teror teror_to_mux_2(
                .a(wire4[l*2 +:2]),
                .b(wire3[l*2 +:2]),
                .out(out[l*2 +:2])  
                );
        end
    endgenerate
endmodule
module six_mux(   input [ 11 :0 ] a , input [ 11 :0 ] b , input [ 11 :0 ] c  , input [1:0] choose , output [11:0]out ); //left-1 mid0 right1
    wire [1:0] po ,ze ,ne; 
    decoder decoder_to_mux( .a(choose) ,.po(po ) ,.ze( ze ) ,.ne(ne));
    genvar l;
    wire [11:0] wire1 ,wire2 ,wire3,wire4;
    generate
        for( l = 0 ; l< 6 ; l = l+1 )begin : and_or_array
            terand terand_to_mux_po(
                .a(c[l*2 +:2]),
                .b(po), 
                .out(wire1[l*2 +:2])    
                );
            terand terand_to_mux_mid(
                .a(b[l*2 +:2]),
                .b(ze),
                .out(wire2[l*2 +:2] )   
                );
            terand terand_to_mux_ne(
                .a(a[l*2 +:2] ),
                .b(ne), 
                .out(wire3[l*2 +:2])    
                );
            teror teror_to_mux_1(
                .a(wire1[l*2 +:2]),
                .b(wire2[l*2 +:2]),
                .out(wire4[l*2 +:2])
                );
            teror teror_to_mux_2(
                .a(wire4[l*2 +:2]),
                .b(wire3[l*2 +:2]),
                .out(out[l*2 +:2])  
                );
        end
    endgenerate
endmodule
//@ai
module five_bit_to_four_tit( input [4:0] in , output [7:0] out );//413~473 @ai [to_do]
    parameter p = 2'b10, n = 2'b00, o = 2'b01;
    reg [7:0] nonbaltit;
    reg [7:0] ter_ori;
    integer i;
    integer num;
    integer carry_reg;
    integer sum;
    always @(*) begin
        num = in;
        nonbaltit = 8'b0;
        for (i = 0; i < 4; i = i + 1) begin
            nonbaltit[2*i +: 2] = num % 3;
            num = num / 3;
        end
        carry_reg = 0;
        ter_ori = 8'b0;
        for (i = 0; i < 4; i = i + 1) begin
            sum = nonbaltit[2*i +: 2] + carry_reg;
            case (sum)
                0: begin ter_ori[2*i +: 2] = o; carry_reg = 0; end 
                1: begin ter_ori[2*i +: 2] = p; carry_reg = 0; end 
                2: begin ter_ori[2*i +: 2] = n; carry_reg = 1; end 
                3: begin ter_ori[2*i +: 2] = o; carry_reg = 1; end 
                default: begin ter_ori[2*i +: 2] = o; carry_reg = 0; end
            endcase
        end
    end
    assign out = ter_ori;
endmodule
module three_bit_to_three_tit( input [2:0] in , output [5:0] out );
    parameter p = 2'b10, n = 2'b00, o = 2'b01;
    reg [5:0] nonbaltit;
    reg [5:0] ter_ori;
    integer i;
    integer num;
    integer carry_reg;
    integer sum;
    always @(*) begin
        num = in;
        nonbaltit = 6'b0;
        for (i = 0; i < 3; i = i + 1) begin
            nonbaltit[2*i +: 2] = (num % 3) & 2'b11;
            num = num / 3;
        end
        carry_reg = 0;
        ter_ori = 6'b0;
        for (i = 0; i < 3; i = i + 1) begin
            sum = nonbaltit[2*i +: 2] + carry_reg;
            case (sum)
                0: begin ter_ori[2*i +: 2] = o; carry_reg = 0; end 
                1: begin ter_ori[2*i +: 2] = p; carry_reg = 0; end 
                2: begin ter_ori[2*i +: 2] = n; carry_reg = 1; end 
                3: begin ter_ori[2*i +: 2] = o; carry_reg = 1; end 
                default: begin ter_ori[2*i +: 2] = o; carry_reg = 0; end
            endcase
        end
    end
    assign out = ter_ori;
endmodule
module seven_bit_to_six_tit( input [6:0] in , output [11:0] out );
    parameter p = 2'b10, n = 2'b00, o = 2'b01;
    reg [11:0] nonbaltit;
    reg [11:0] ter_ori;
    integer i;
    integer num;
    integer carry_reg;
    integer sum;
    always @(*) begin
        num = { 25'b0 ,in};
        nonbaltit = 12'b0;
        for (i = 0; i < 6; i = i + 1) begin
            nonbaltit[2*i +: 2] = (num % 3 )& 2'b11;
            num = num / 3;
        end
        carry_reg = 0;
        ter_ori = 12'b0;
        for (i = 0; i < 6; i = i + 1) begin
            sum = nonbaltit[2*i +: 2] + carry_reg;
            case (sum)
                0: begin ter_ori[2*i +: 2] = o; carry_reg = 0; end 
                1: begin ter_ori[2*i +: 2] = p; carry_reg = 0; end 
                2: begin ter_ori[2*i +: 2] = n; carry_reg = 1; end 
                3: begin ter_ori[2*i +: 2] = o; carry_reg = 1; end 
                default: begin ter_ori[2*i +: 2] = o; carry_reg = 0; end
            endcase
        end
    end
    assign out = ter_ori;
endmodule
//end @ai
module four_eq( input [7:0] a ,input [7:0]b ,output [1:0] out); 
    parameter p = 2'b10, n = 2'b00, o = 2'b01;
    wire [ 7:0] a_n;
    wire [7:0] any_array;
    wire [7:0] is_z_array;
    wire [1:0] four_eq_tmp1 , four_eq_tmp2;
    onot onot_four_eq_a_n1( .a(a[1:0] ) ,.out( a_n[1:0]) );
    onot onot_four_eq_a_n2( .a(a[3:2] ) ,.out( a_n[3:2]) );
    onot onot_four_eq_a_n3( .a(a[5:4] ) ,.out( a_n[5:4]) );
    onot onot_four_eq_a_n4( .a(a[7:6] ) ,.out( a_n[7:6]) );
    terany terany_four_eq_anyarr1( .a(a_n[1:0] ) ,.b(b[1:0]) ,.out( any_array[1:0]));
    terany terany_four_eq_anyarr2( .a(a_n[3:2] ) ,.b(b[3:2]) ,.out( any_array[3:2]));
    terany terany_four_eq_anyarr3( .a(a_n[5:4] ) ,.b(b[5:4]) ,.out( any_array[5:4]));
    terany terany_four_eq_anyarr4( .a(a_n[7:6] ) ,.b(b[7:6]) ,.out( any_array[7:6]));
    isz isz_four_eq_is_z1( .a(  any_array[1:0]) ,.out( is_z_array[1:0]) );
    isz isz_four_eq_is_z2( .a(  any_array[3:2]) ,.out( is_z_array[3:2]) );
    isz isz_four_eq_is_z3( .a(  any_array[5:4]) ,.out( is_z_array[5:4]) );
    isz isz_four_eq_is_z4( .a(  any_array[7:6]) ,.out( is_z_array[7:6]) );
    terand terand_four_eq_tmp1( .a( is_z_array[1:0]) ,.b(is_z_array[3:2])  ,.out( four_eq_tmp1));
    terand terand_four_eq_tmp2( .a( four_eq_tmp1 ) ,.b(is_z_array[5:4])  ,.out( four_eq_tmp2));
    terand terand_four_eq_tmp3( .a( four_eq_tmp2) ,.b(is_z_array[7:6])  ,.out( out));
endmodule
module tercom_to( input [41:0] a, input [41:0] b, output [1:0] out ); //n a>b o a=b p a<b
    //靈感來源:http://nfudee.nfu.edu.tw/ezfiles/43/1043/img/326/dc17.pdf  
    parameter p = 2'b10, n = 2'b00, o = 2'b01;

    wire [1:0] ena [0:21];
    assign ena[0] = p;

    wire [1:0] ena_any [0:20];
    wire [1:0] sum   [0:21];
    assign sum[0] = o;

    genvar i;
    generate
        for (i = 0; i < 21; i = i + 1) begin : cmp_loop
            localparam integer idx = 20 - i;

            wire [1:0] a_trit = a[idx*2 +: 2];
            wire [1:0] b_trit = b[idx*2 +: 2];

            wire [1:0] onot_a;
            onot onot_inst(.a(a_trit), .out(onot_a));

            wire [1:0] any_raw;
            terany terany_inst(.a(b_trit), .b(onot_a), .out(any_raw));

            wire [1:0] clu_ena;
            clu clu_inst(.a(ena[i]), .out(clu_ena));

            ternmul ternmul_en(.a(any_raw), .b(clu_ena), .out(ena_any[i]));

            wire [1:0] isz_ena_any;
            isz isz_inst(.a(ena_any[i]), .out(isz_ena_any));

            terand terand_ena(.a(isz_ena_any), .b(ena[i]), .out(ena[i+1]));

            tersum tersum_inst(.a(sum[i]), .b(ena_any[i]), .out(sum[i+1]));
        end
    endgenerate
    assign out = sum[21]; 
endmodule
//因為我不確定哪種logic gate會先出來 考慮( 1 0 ) = + 0 , + - 兩種
//與用二進位測試這三種版本,  用para = 00            01    10



/* verilator lint_on DECLFILENAME */
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNUSEDPARAM */
/* verilator lint_on UNOPTFLAT */
/* verilator lint_on WIDTHEXPAND */
/* verilator lint_on WIDTHTRUNC */
/* verilator lint_on CASEINCOMPLETE */


