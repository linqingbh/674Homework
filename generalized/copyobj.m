function b = copyobj(a)
   b = eval(class(a(1))+".empty("+string(length(a))+",0)");  %create default object of the same class as a. one valid use of eval
   for j = 1:length(a)
       b = eval(class(a(j)));
       for p =  properties(a(1)).'  %copy all public properties
          try   %may fail if property is read-only
             b.(p) = a(j).(p);
          catch
             warning('failed to copy property: %s', cell2mat(p));
          end
       end
   end
   output = [output;b];
end