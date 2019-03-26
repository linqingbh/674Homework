function points = draw_circle(base,radius,direction,l1,l2,ref)
    if ~exist('direction','var')||isempty(direction),direction = 1;end
    if ~exist('l1','var')||isempty(l1),l1=0;l2=2*pi;end
    if ~exist('ref','var')||isempty(ref),ref=0;end
    
    if (((l1<l2) || approx(l1,l2)) && (direction>=0))
        rad = l1:0.01:l2;
    elseif (((l1>l2) || approx(l1,l2)) && (direction<0))
        rad = fliplr(l2:0.01:l1);
    elseif ((l1<l2) && (direction<0))
        rad = fliplr([l2:0.01:pi,-pi:0.01:l1]);
    elseif ((l1>l2) && (direction>=0))
        rad = [l1:0.01:pi,-pi:0.01:l2];
    end
    
    points = base + radius*[cos(rad+ref);sin(rad+ref);zeros(size(rad))];
end