function jacobian = numerical_jacobian(f,x_star,epsilon)
    if ~exist('epsilon','var'),epsilon = 0.01;end
    
    y_star = f(x_star);

    jacobian = zeros(length(y_star),length(x_star));
    for i = 1:length(x_star)
        perturbance = zeros(length(x_star),1);
        perturbance(i) = epsilon;
        jacobian(:,i) = numdiff1(f,x_star,perturbance);
    end
end