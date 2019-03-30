function jacobian = numerical_jacobian(f,x_star,epsilon,x_indexes,y_indexes,y_is_angle)

    y_star = f(x_star);
    cheat = true;
    if ~exist('epsilon','var') || isempty(epsilon),epsilon = 0.01;end
    if ~exist('x_indexes','var') || isempty(x_indexes),x_indexes = 1:length(x_star);end
    if ~exist('y_indexes','var') || isempty(y_indexes),y_indexes = 1:length(y_star);cheat = false;end
    if ~exist('y_is_angle','var') || isempty(y_is_angle),y_is_angle = false(size(y_star));end
    
%     for i = 1:length(x_star)
%         if x_star(i) == 0
%             x_star(i) = 100;
%         end
%     end

    jacobian = zeros(length(y_star),length(x_star));
    for i = 1:length(x_star)
        perturbance = zeros(length(x_star),1);
        perturbance(i) = epsilon;
        jacobian(:,i) = numdiff1(f,x_star,perturbance,y_is_angle);
    end

    
    jacobian = jacobian(y_indexes,x_indexes);
    
    if cheat
        jacobian(end-2:end,end-2:end) = eye(3);
        jacobian(end,end) = 1;
    end
    
end