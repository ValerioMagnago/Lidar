function [data] = test(n)
    coder.varsize('data', [], [0 1]);
    data = ones(1,0);
    for i=1:n
        data = [i, data];
    end    
end


