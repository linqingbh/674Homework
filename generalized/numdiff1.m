function df_dx = numdiff1(f,x,epsilon,is_angle)

    if ~exist('epsilon','var') || isempty(epsilon),epsilon = 0.001;end
    if ~exist('is_angle','var') || isempty(is_angle),is_angle = false;end
    
    %index = find(epsilon);

    df_dx = (controllers.get_error(f(x),f(x+epsilon),is_angle))/epsilon(logical(epsilon));
%     while 1
%         df_dx_check = (f(x+epsilon)-f(x))/epsilon(index);
%         epsilon = epsilon*0.1;
%         df_dx = (f(x+epsilon)-f(x))/epsilon(index);
%         if abs(percent_differance(df_dx_check,df_dx)) < 0.01
%             break
%         end
%     end
    
end