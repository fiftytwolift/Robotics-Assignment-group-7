for i = 1:10
    q = 2*pi*rand(1,5);
    Origianl = calculate_Jacobian(q);
    diff_Version = calculate_Jacobian_Differentiate_Method(q);
    diff = vpa(Origianl(1:3,:)-diff_Version,2)
    if ( abs((Origianl(1:3,:)-diff_Version)) < 0.01)
        disp("the two results are the same");
    else
        disp("the two results are not the same");
    end
end

