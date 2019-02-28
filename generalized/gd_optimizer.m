function x_star = gd_optimizer(fun,initial,kappa,min_dJ_dx,epsilon)
    if ~exist('kappa','var'),kappa = 10^-6;end
    if ~exist('min_dJ_dx','var'),min_dJ_dx = 10^-6;end
    if ~exist('epsilon','var'),epsilon = 10^-6;end
    
    x_star = initial;
    dJ_dx = inf(size(x_star));
    while any(dJ_dx>min_dJ_dx)
        dJ_dx = numdiff1(fun,x_star,epsilon);
        x_star = x_star - kappa*dJ_dx;
    end
    
end