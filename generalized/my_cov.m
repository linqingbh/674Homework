function covariance_matrix = my_cov(fun,x)
    if ~exist('iterations','var'),iterations = 1000;end
            
    for i = 1:iterations
        measurements(:,i) = fun(randn(size(x))); %#ok<AGROW>
    end
    covariance_matrix = corrcoef(measurements.');
%     for i = 1:iterations
%         measurements(:,i) = fun(input); %#ok<AGROW>
%     end
%     covariance_matrix = cov(measurements.');
end