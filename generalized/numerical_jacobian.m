function jacobian = numerical_jacobian(f,x_star)
    
    y_star = f(x_star);

    jacobian = zeros(length(y_star),length(x_star));
    for i = 1:length(x_star)
        epsilon = zeros(length(x_star),1);
        epsilon(i) = 0.01;
        jacobian(:,i) = numdiff1(f,x_star,epsilon);
    end
end