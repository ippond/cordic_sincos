`timescale 1ns/100ps
module cordic_sincos
#(
parameter C_ANGLE_WIDTH     = 16 ,
parameter C_SINCOS_WIDTH    = 16 ,
parameter C_ITERATION_TIMES = 16 ,
parameter C_PIPELINE        = 1
)
(
input                                       I_clk     ,
input       signed   [C_ANGLE_WIDTH-1:0]    I_angle   ,
input                                       I_angle_v ,
output reg           [C_SINCOS_WIDTH-1:0]   O_sin     ,
output reg           [C_SINCOS_WIDTH-1:0]   O_cos     ,
output reg                                  O_sincos_v
);

localparam C_SHIFT_WIDTH = F_cal_width(C_SINCOS_WIDTH);
localparam C_ITER_WIDTH = F_cal_width(C_ITERATION_TIMES);

integer S_i;
wire signed [C_ANGLE_WIDTH-1:0] S_halfpi_pos = 3.141592653589793 * (2**(C_ANGLE_WIDTH-4));
wire signed [C_ANGLE_WIDTH-1:0] S_halfpi_neg = -3.141592653589793 * (2**(C_ANGLE_WIDTH-4));
wire [C_ANGLE_WIDTH-1:0] S_pi_pos = 3.141592653589793 * (2**(C_ANGLE_WIDTH-3));
wire [C_ANGLE_WIDTH-1:0] S_pi_neg = -3.141592653589793 * (2**(C_ANGLE_WIDTH-3));
reg S_pi_id1 = 0;
reg S_pi_id2 = 0;
reg S_pi_id3 = 0;
reg [C_ANGLE_WIDTH-1:0] S_angle = 0;
//pipeline
wire [C_ANGLE_WIDTH-1:0] S_atan_step_pipeline [31:0];
reg [C_SINCOS_WIDTH-1:0] S_data_i_shift [C_ITERATION_TIMES:0];
reg [C_SINCOS_WIDTH-1:0] S_data_q_shift [C_ITERATION_TIMES:0];
reg [C_ANGLE_WIDTH-1:0] S_angle_pipeline [C_ITERATION_TIMES:0];
reg [C_ITERATION_TIMES+1:0] S_angle_v_slr = 0;
reg [C_ITERATION_TIMES*3+2:0] S_quad_slr = 0;
//non-pipeline
wire [C_ANGLE_WIDTH*32-1:0] S_atan_step;
reg [2:0] S_quad = 0;
reg S_iter_v = 0;
reg [C_ITER_WIDTH-1:0] S_iter_cnt = 0;
reg [C_ITER_WIDTH-1:0] S_iter_cnt_d = 0;
reg S_iter_over = 0;
reg S_angle_v = 0;
reg S_angle_v_d = 0;
reg signed [C_SINCOS_WIDTH-1:0] S_data_i_iter = 0;
reg signed [C_SINCOS_WIDTH-1:0] S_data_q_iter = 0;
reg [C_ANGLE_WIDTH*32-1:0] S_atan_step_slr;
reg [C_ANGLE_WIDTH-1:0] S_angle_iter = 0;


always @(posedge I_clk)
begin
    S_pi_id1 <= I_angle >= S_halfpi_pos;
    S_pi_id2 <= I_angle <= S_halfpi_neg;
    S_pi_id3 <= (I_angle > S_halfpi_neg) && I_angle[C_ANGLE_WIDTH-1];
    S_angle <= I_angle;
    if(S_pi_id1)
        S_angle_pipeline[0] <= S_pi_pos - S_angle;
    else if(S_pi_id2 && !S_pi_id3)
        S_angle_pipeline[0] <= S_pi_pos + S_angle;
    else
        S_angle_pipeline[0] <= S_angle;
    S_data_i_shift[0] <= 0.6073 * (2**(C_SINCOS_WIDTH-2));
    S_data_q_shift[0] <= 'd0;
    S_quad_slr <= {S_quad_slr[C_ITERATION_TIMES*3-1:0],S_pi_id1,S_pi_id2,S_pi_id3};
    S_angle_v_slr <= {S_angle_v_slr[C_ITERATION_TIMES:0],I_angle_v};
end

generate
if(C_PIPELINE==1)
begin:pipeline

always @(posedge I_clk)
begin
    O_sincos_v <= S_angle_v_slr[C_ITERATION_TIMES+1];
    if(S_angle_v_slr[C_ITERATION_TIMES+1])
    begin
        case(S_quad_slr[C_ITERATION_TIMES*3+2:C_ITERATION_TIMES*3])
        3'b100:
        begin
            O_sin <= S_data_q_shift[C_ITERATION_TIMES];
            O_cos <= ~S_data_i_shift[C_ITERATION_TIMES]+'d1;
        end
        3'b010:
        begin
            O_sin <= ~S_data_q_shift[C_ITERATION_TIMES]+'d1;
            O_cos <= ~S_data_i_shift[C_ITERATION_TIMES]+'d1;
        end
        default:
        begin
            O_sin <= S_data_q_shift[C_ITERATION_TIMES];
            O_cos <= S_data_i_shift[C_ITERATION_TIMES];
        end
        endcase
    end
end

end
else
begin:non_pipeline

always @(posedge I_clk)
begin
    O_sincos_v <= S_iter_over;
    if(S_iter_over)
    begin
        case(S_quad)
        3'b100:
        begin
            O_sin <= S_data_q_iter;
            O_cos <= ~S_data_i_iter+'d1;
        end
        3'b010:
        begin
            O_sin <= ~S_data_q_iter+'d1;
            O_cos <= ~S_data_i_iter+'d1;
        end
        default:
        begin
            O_sin <= S_data_q_iter;
            O_cos <= S_data_i_iter;
        end
        endcase
    end
end

end
endgenerate

//--------------------------------
//pipeline
//--------------------------------
assign S_atan_step_pipeline[0]  = 0.785398163397448 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[1]  = 0.463647609000806 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[2]  = 0.244978663126864 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[3]  = 0.124354994546761 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[4]  = 0.062418809995957 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[5]  = 0.031239833430268 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[6]  = 0.015623728620477 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[7]  = 0.007812341060101 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[8]  = 0.003906230131967 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[9]  = 0.001953122516479 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[10] = 0.000976562189559 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[11] = 0.000488281211195 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[12] = 0.000244140620149 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[13] = 0.000122070311894 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[14] = 0.000061035156174 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[15] = 0.000030517578116 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[16] = 0.000015258789061 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[17] = 0.000007629394531 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[18] = 0.000003814697266 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[19] = 0.000001907348633 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[20] = 0.000000953674316 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[21] = 0.000000476837158 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[22] = 0.000000238418579 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[23] = 0.000000119209290 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[24] = 0.000000059604645 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[25] = 0.000000029802322 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[26] = 0.000000014901161 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[27] = 0.000000007450581 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[28] = 0.000000003725290 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[29] = 0.000000001862645 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[30] = 0.000000000931323 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step_pipeline[31] = 0.000000000465661 * (2**(C_ANGLE_WIDTH-3));

genvar gen_i;
generate
for(gen_i=0;gen_i<C_ITERATION_TIMES;gen_i=gen_i+1)
begin:parallel_iteration

always @(posedge I_clk)
begin
    if(S_angle_pipeline[gen_i][C_SINCOS_WIDTH-1])
    begin
        S_data_i_shift[gen_i+1] <= S_data_i_shift[gen_i] + {{gen_i{S_data_q_shift[gen_i][C_SINCOS_WIDTH-1]}},S_data_q_shift[gen_i][C_SINCOS_WIDTH-1:gen_i]};
        S_data_q_shift[gen_i+1] <= S_data_q_shift[gen_i] - {{gen_i{S_data_i_shift[gen_i][C_SINCOS_WIDTH-1]}},S_data_i_shift[gen_i][C_SINCOS_WIDTH-1:gen_i]};
        S_angle_pipeline[gen_i+1] <= S_angle_pipeline[gen_i] + S_atan_step_pipeline[gen_i];
    end
    else
    begin
        S_data_i_shift[gen_i+1] <= S_data_i_shift[gen_i] - {{gen_i{S_data_q_shift[gen_i][C_SINCOS_WIDTH-1]}},S_data_q_shift[gen_i][C_SINCOS_WIDTH-1:gen_i]};
        S_data_q_shift[gen_i+1] <= S_data_q_shift[gen_i] + {{gen_i{S_data_i_shift[gen_i][C_SINCOS_WIDTH-1]}},S_data_i_shift[gen_i][C_SINCOS_WIDTH-1:gen_i]};
        S_angle_pipeline[gen_i+1] <= S_angle_pipeline[gen_i] - S_atan_step_pipeline[gen_i];
    end
end

end
endgenerate

//-------------------------------
//non-pipeline
//-------------------------------
assign S_atan_step[C_ANGLE_WIDTH*0 +:C_ANGLE_WIDTH]  = 0.785398163397448 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*1 +:C_ANGLE_WIDTH]  = 0.463647609000806 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*2 +:C_ANGLE_WIDTH]  = 0.244978663126864 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*3 +:C_ANGLE_WIDTH]  = 0.124354994546761 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*4 +:C_ANGLE_WIDTH]  = 0.062418809995957 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*5 +:C_ANGLE_WIDTH]  = 0.031239833430268 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*6 +:C_ANGLE_WIDTH]  = 0.015623728620477 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*7 +:C_ANGLE_WIDTH]  = 0.007812341060101 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*8 +:C_ANGLE_WIDTH]  = 0.003906230131967 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*9 +:C_ANGLE_WIDTH]  = 0.001953122516479 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*10+:C_ANGLE_WIDTH] = 0.000976562189559 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*11+:C_ANGLE_WIDTH] = 0.000488281211195 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*12+:C_ANGLE_WIDTH] = 0.000244140620149 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*13+:C_ANGLE_WIDTH] = 0.000122070311894 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*14+:C_ANGLE_WIDTH] = 0.000061035156174 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*15+:C_ANGLE_WIDTH] = 0.000030517578116 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*16+:C_ANGLE_WIDTH] = 0.000015258789061 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*17+:C_ANGLE_WIDTH] = 0.000007629394531 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*18+:C_ANGLE_WIDTH] = 0.000003814697266 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*19+:C_ANGLE_WIDTH] = 0.000001907348633 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*20+:C_ANGLE_WIDTH] = 0.000000953674316 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*21+:C_ANGLE_WIDTH] = 0.000000476837158 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*22+:C_ANGLE_WIDTH] = 0.000000238418579 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*23+:C_ANGLE_WIDTH] = 0.000000119209290 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*24+:C_ANGLE_WIDTH] = 0.000000059604645 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*25+:C_ANGLE_WIDTH] = 0.000000029802322 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*26+:C_ANGLE_WIDTH] = 0.000000014901161 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*27+:C_ANGLE_WIDTH] = 0.000000007450581 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*28+:C_ANGLE_WIDTH] = 0.000000003725290 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*29+:C_ANGLE_WIDTH] = 0.000000001862645 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*30+:C_ANGLE_WIDTH] = 0.000000000931323 * (2**(C_ANGLE_WIDTH-3));
assign S_atan_step[C_ANGLE_WIDTH*31+:C_ANGLE_WIDTH] = 0.000000000465661 * (2**(C_ANGLE_WIDTH-3));

always @(posedge I_clk)
begin
    S_angle_v   <= I_angle_v;
    S_angle_v_d <= S_angle_v;
    if(S_angle_v)
    begin
        S_quad <= {S_pi_id1,S_pi_id2,S_pi_id3};
    end
    
    if(S_angle_v)
        S_iter_v <= 'd1;
    else if(S_iter_cnt == C_ITERATION_TIMES)
        S_iter_v <= 'd0;
    
    if(S_iter_v)    
        S_iter_cnt <= S_iter_cnt + 'd1;
    else
        S_iter_cnt <= 'd0;
    S_iter_over <= S_iter_cnt == C_ITERATION_TIMES;
    S_iter_cnt_d <= S_iter_cnt;
end

always @(posedge I_clk)
begin
    if(S_angle_v_d)
    begin
        S_data_i_iter <= 0.6073 * (2**(C_SINCOS_WIDTH-2));
        S_data_q_iter <= 'd0;
        S_angle_iter <= S_angle_pipeline[0];
        S_atan_step_slr <= S_atan_step;
    end
    else
    begin
        if(S_angle_iter[C_ANGLE_WIDTH-1])
        begin
            S_data_i_iter <= S_data_i_iter + (S_data_q_iter>>>S_iter_cnt_d);
            S_data_q_iter <= S_data_q_iter - (S_data_i_iter>>>S_iter_cnt_d);
            S_angle_iter <= S_angle_iter + S_atan_step_slr[C_ANGLE_WIDTH-1:0];
        end
        else
        begin
            S_data_i_iter <= S_data_i_iter - (S_data_q_iter>>>S_iter_cnt_d);
            S_data_q_iter <= S_data_q_iter + (S_data_i_iter>>>S_iter_cnt_d);
            S_angle_iter <= S_angle_iter - S_atan_step_slr[C_ANGLE_WIDTH-1:0];
        end
        S_atan_step_slr <= S_atan_step_slr >> C_ANGLE_WIDTH;
    end
end


function integer F_cal_width;
input integer I_data;
integer i;
begin
    for(i=1;(2**i)<=I_data;i=i+1)
    F_cal_width = i;
    F_cal_width = i;
end
endfunction

endmodule


