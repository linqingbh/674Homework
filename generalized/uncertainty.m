function output = uncertainty(input,uncertainty,keys)    
            
            % Scale the uncertianty
            scale = uncertainty.random.*(rand(1,length(uncertainty.random)).*2-1);
            
            % If we're adjusting the parameters
            if isstruct(input)
                
                output = input;
                for i = 1:length(keys)
                    % Only impliment the uncertianty for the variables given.
                    variable = keys{i};
                    code = ['output.',variable,' = input.',variable,'+ scale(i) + uncertainty.bias(i);'];
                    eval(code)
%                     % This is to see the new values if needed
%                     disp(variable)
%                     eval(['disp(input.',variable,')'])
%                     eval(['disp(output.',variable,')'])
                end
                
             else % if we're adjusting any other number
                
                output = zeros(size(input));
                for i = 1:length(input)
                    % Only make uncertian if the key tells us to.
                    if keys(i)
                        output(i) = input(i) + scale(i) + uncertainty.bias(i);
                    else
                        output(i) = input(i);
                    end
                end
                
            end
        end