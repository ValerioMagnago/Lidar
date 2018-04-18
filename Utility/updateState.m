function state = updateState(state,v,omega,dt)
%UPDATESTATE Summary of this function goes here
%   Detailed explanation goes here

if abs(omega)<0.01
    % Stright movement
    state(1) = state(1) + v*dt*cos(state(3));
    state(2) = state(2) + v*dt*sin(state(3));
    state(3) = state(3) + omega*dt;
else
    % Mooving on arc
    R = v/omega;
    dtheta = omega*dt;

    dy_rel = (R-R*cos(dtheta));
    dx_rel = R*sin(dtheta);

    rot = [cos(state(3)), sin(-state(3)); sin(state(3)), cos(state(3))];
    spost = rot*[dx_rel;dy_rel];
    state(1) = state(1) + spost(1);
    state(2) = state(2) + spost(2);
    state(3) = state(3) + dtheta;
end
    
end

