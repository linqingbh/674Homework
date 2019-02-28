function points = function_generator(input,period,amplitude,offset,phase_delay,t)
    switch input
        case 'sine'
            points = amplitude.*sin(t.*(2.*pi)./period - phase_delay) + offset;
        case 'ramp'
            points = amplitude/2.*sawtooth(t.*(2.*pi)./period  - phase_delay) + offset + amplitude/2;
        case 'square'
            points = amplitude.*square(t.*(2.*pi)./period  - phase_delay) + offset;
        otherwise
            points = zeros(size(t))+offset;     
    end
end

