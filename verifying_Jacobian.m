for h = 0:10
    for i = 0:10
        for j = 0:10
            for k = 0:10
                q = 2*pi*([h/10 i/10, j/10, k/10, 0]);
                Origianl = calculate_Jacobian(q);
                diff_Version = calculate_Jacobian_Differentiate_Method(q);
                diff = vpa(Origianl(1:3,:)-diff_Version,2)
                if ( abs((Origianl(1:3,:)-diff_Version)) < 0.01)
                    disp("the two results are the same");
                else                   
                    disp("the two results are not the same");
                    return
                end
            end
        end
    end
end

