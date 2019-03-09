function vector_rotated = get_rotation(phi,theta,psi,type,vector)
    if ~exist('vector','var'),vector=eye(3);end
    
    I = eye(3);
    R_bv2 = [1,0,0;
             0,cos(phi),-sin(phi);
             0,sin(phi),cos(phi)];
    R_v2v1 = [cos(theta),0,sin(theta);
              0,1,0;
              -sin(theta),0,cos(theta)];
    R_v1v = [cos(psi),-sin(psi),0;
             sin(psi),cos(psi),0;
             0,0,1];
    R_NED_ENU = [0,1,0;
                 1,0,0;
                 0,0,-1];
    R_bv12 = [1,sin(phi)*tan(theta),cos(phi)*tan(theta);
              0,cos(phi),-sin(phi);
              0,sin(phi)*sec(theta),cos(phi)*sec(theta)];
    switch type
        case 'b->b'
            R = I;
        case 'b->v2'
            R = R_bv2;
        case 'b->v1'
            R = R_v2v1*R_bv2;
        case 'b->v'
            R = R_v1v*R_v2v1*R_bv2;
        case 'v2->b'
            R = R_bv2.';
        case 'v2->v2'
            R = I;
        case 'v2->v1'
            R = R_v2v1;
        case 'v2->v'
            R = R_v1v*R_v2v1;
        case 'v1->b'  
            R = R_bv2.'*R_v2v1.';
        case 'v1->v2'
            R = R_v2v1.';
        case 'v1->v1'
            R = I;
        case 'v1->v'
            R = R_v1v;
        case 'v->b'
            R = R_bv2.'*R_v2v1.'*R_v1v.';
        case 'v->v2'
            R = R_v2v1.'*R_v1v.';
        case 'v->v1'
            R = R_v1v.';
        case 'v->v'
            R = I;
        case 'NED->ENU'
            R = R_NED_ENU;
        case 'ENU->NED'
            R = R_NED_ENU.';
        case 'v12->b'
            R = R_bv12.';
        case 'b->v12'
            R = R_bv12;
    end
    
    vector_rotated = R*vector;
end