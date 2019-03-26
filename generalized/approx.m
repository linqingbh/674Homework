function output = approx(value1,value2,type,plus_or_minus)
    if ~exist('type','var') || isempty(type),type = 1;end
    if ~exist('plus_or_minus','var') || isempty(plus_or_minus)
        if type == 1
            plus_or_minus = 0.1;
        else
            plus_or_minus = 10^-5;
        end
    end
    output = true(size(value1));
    for i = 1:size(value1,1)
        for j = 1:size(value2,2)
            if type == 1
                diff = abs(percent_differance(value1(i,j),value2(i,j)));
            else
                diff = abs(value1(i,j)-value2(i,j));
            end
            output(i,j) = diff <= plus_or_minus;
        end
    end
end