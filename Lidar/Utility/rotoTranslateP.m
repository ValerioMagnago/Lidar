function [ p_roto_trasl ] = rotoTranslateP( P, theta, T )
%ROTOTRANSLATEP Summary of this function goes here
%   Detailed explanation goes here

R = [cos(theta),-sin(theta);sin(theta),cos(theta)];

p_roto_trasl = R*P + T;

end

