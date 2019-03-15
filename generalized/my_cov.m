function covariance_matrix = my_cov(fun,x)
    if ~exist('iterations','var'),iterations = 1000;end
            
    for i = 1:iterations
        [measurements(:,i),disturbances(:,i)] = fun(randn(size(x))); %#ok<AGROW>
    end
    covariance_matrix = cov([measurements.',disturbances(1:3,:).']);
    corrilation_matrix = corrcoef(measurements.');
    covariance_matrix(1:12,1:12) = corrilation_matrix;
%     for i = 1:iterations
%         measurements(:,i) = fun(input); %#ok<AGROW>
%     end
%     covariance_matrix = cov(measurements.');
end