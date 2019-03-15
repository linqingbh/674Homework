function jacobian = numerical_jacobian(f,x_star,epsilon,y_indexes,x_indexes)
    y_star = f(x_star);

    if ~exist('epsilon','var'),epsilon = 0.01;end
    if ~exist('x_indexes','var'),x_indexes = 1:length(x_star);end
    if ~exist('y_indexes','var'),y_indexes = 1:length(y_star);end

%     tic
    jacobian = zeros(length(y_star),length(x_star));
    for i = 1:length(x_star)
        perturbance = zeros(length(x_star),1);
        perturbance(i) = epsilon;
%         tic
        jacobian(:,i) = numdiff1(f,x_star,perturbance);
%         toc
    end
%     toc
    
    jacobian = jacobian(y_indexes,x_indexes);
end