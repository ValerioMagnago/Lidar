function [  ] = use_lidar(  )
filename = which(mfilename);
[pathstr,~,~] = fileparts(filename);

addpath(genpath([pathstr,'/Utility']));

end
