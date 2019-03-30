function F = saturate(F,sat_lim)
    if length(F) ~= length(sat_lim(:,1))
        sat_lim(1:length(F),1) = sat_lim(1);
        if length(sat_lim(1,:)) == 2
            sat_lim(1:length(F),2) = sat_lim(1,2);
        end
    end
    for i = 1:length(F)
        if length(sat_lim(i,:))==1,sat_lim(i,[1,2]) = [-sat_lim(i),sat_lim(i)];end
        sat_lim(i,:) = sort(sat_lim(i,:));
        if F(i) >= sat_lim(i,2)
            F(i) = sat_lim(i,2);
            warning('SATURATING: High saturation limit reached.')
        elseif F(i) <= sat_lim(i,1)
            F(i) = sat_lim(i,1);
            warning('SATURATING: Low saturation limit reached.')
        end
    end
end