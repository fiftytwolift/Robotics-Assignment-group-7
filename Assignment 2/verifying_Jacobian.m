for h = 1:3
    for i = 1:3
        for j = 1:3
            for k = 1:3
                q = 2*pi*([h/10 i/10, j/10, k/10, 0]);
                Origianl = calculate_Jacobian(q);
                Origianl2 = Origianl + 1e15;
                Origianl = Origianl2 - 1e15
                diff_Version = calculate_Jacobian_Differentiate_Method(q);
                diff_Version2 = diff_Version + 1e15;
                diff_Version = diff_Version2 - 1e15
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

