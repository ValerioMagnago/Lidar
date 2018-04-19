function y = stack(command, varargin)
persistent data;
if isempty(data)
    data = ones(1,0);
end
y = 0;
switch (command)
    case {'init'}
        
        
    case {'pop'}
        y = data(1);
        data = data(2:size(data, 2));
    case {'push'}
        
    case {'get'}
        y = data(varargin{1});
    case {'length'}
        y = length(data);
    otherwise
        assert(false, ['Wrong command: ', command]);
end
end