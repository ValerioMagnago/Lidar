function [  ] = plotElipse( state, P, figura_id, Tag )
%PLOTELIPSE Summary of this function goes here
%   Detailed explanation goes here

discretizz_ang = linspace(0,2*pi,30);
[V,D] = eig(P(1:2,1:2,1));
D = abs(D);
ellisse_x = 3*(V(1,1)*sqrt(D(1,1))*cos(discretizz_ang) + V(1,2) * sqrt(D(2,2)) * sin(discretizz_ang));
ellisse_y = 3*(V(2,1)*sqrt(D(1,1))*cos(discretizz_ang) + V(2,2) * sqrt(D(2,2)) * sin(discretizz_ang));

figure(figura_id);

% selectedObjs = findobj('Tag','incElips');
% for kk=1:numel(selectedObjs)
%     delete(selectedObjs(kk));
% end

plot(ellisse_x + state(1),ellisse_y + state(2),'b','Tag',Tag); %'incElips');


end

