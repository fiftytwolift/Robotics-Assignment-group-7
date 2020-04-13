function [output_] = convert_diff_t(input_,state_)
    %DiffArray = ["diff(x(t), t)","diff(y(t), t)","diff(z(t), t)","diff(a(t), t)","diff(b(t), t)","diff(c(t), t)","diff(x_d(t), t)","diff(y_d(t), t)","diff(z_d(t), t)","diff(a_d(t), t)","diff(b_d(t), t)","diff(c_d(t), t)"];
    DiffArray = ["diff(Q1_(t), t)","diff(Q2_(t), t)","diff(Q3_(t), t)","diff(Q4_(t), t)","diff(Q5_(t), t)","diff(c(t), t)","diff(x_d(t), t)","diff(y_d(t), t)","diff(z_d(t), t)","diff(a_d(t), t)","diff(b_d(t), t)","diff(c_d(t), t)"];
    TimeArray = ["Q1_d", "Q2_d", "Q3_d", "Q4_d", "Q5_d", "c_d(t)", "x_dd(t)", "y_dd(t)", "z_dd(t)", "a_dd(t)", "b_dd(t)", "c_dd(t)"];
    if state_ %convert from diff to t
        for i = 1:DiffArray.length
            input_ = subs(input_,str2sym(DiffArray(i)),str2sym(TimeArray(i)));
        end
    else
        for i = 1:TimeArray.length
            input_ = subs(input_,str2sym(TimeArray(i)),str2sym(DiffArray(i)));
        end
    end
    output_ = input_;
end