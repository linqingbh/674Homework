function df_dx = numdiff1(f,x,epsilon)

    if ~exist('epsilon','var'),epsilon = 0.001;end
    
    %index = find(epsilon);

    df_dx = (f(x+epsilon)-f(x))/epsilon(logical(epsilon));
%     while 1
%         df_dx_check = (f(x+epsilon)-f(x))/epsilon(index);
%         epsilon = epsilon*0.1;
%         df_dx = (f(x+epsilon)-f(x))/epsilon(index);
%         if abs(percent_differance(df_dx_check,df_dx)) < 0.01
%             break
%         end
%     end
    
end