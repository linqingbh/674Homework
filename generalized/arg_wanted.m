function varargout = arg_wanted(fun,indexes)
    possible_out = nargout(fun);
    if possible_out <= 0
        error('Function Incorrectly Formated. Check for varargout or no output.')
    end
    [output{1:possible_out}] = fun();
    varargout = {output{indexes}};
end