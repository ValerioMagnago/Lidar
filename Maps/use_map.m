function [  ] = use_map(  )
filename = which(mfilename);
[pathstr,~,~] = fileparts(filename);

addpath(genpath([pathstr,'/Utility']));
addpath(pathstr);
end
