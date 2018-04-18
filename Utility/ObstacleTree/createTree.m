function [ obstaclesStruct ] = createTree( obstacles, xRange, yRange)
%CREATETREE Summary of this function goes here
%   Detailed explanation goes here

for index = 1:numel(obstacles(:,1))
    p = obstacles{index,1};
    objCenter(:,index) = p;
    ptsBBox = obstacles{index,3};
    x_lim = [min(ptsBBox(1,:)),max(ptsBBox(1,:))];
    y_lim = [min(ptsBBox(2,:)),max(ptsBBox(2,:))];
    objBB(:,:,index) = [x_lim;y_lim];
end

orient = 1;
objActive = 1:size(objBB,3);

BB_range = [xRange;yRange];
id = 0;
padre = 0;


[ BB_range,Leaf,Padre, id ] = splitBB( BB_range, objActive,orient, objBB, objCenter, id,padre);
id = id(2:end);

% Create structure
obstaclesTree.Leaf    = zeros(numel(Leaf),1);
obstaclesTree.limits  = zeros(2,2,numel(Leaf));
obstaclesTree.Sons    = cell(numel(Leaf),1);

for i = id
    limits = BB_range(:,:,i);
    figli = find(Padre==i);
    obstaclesTree.Sons{i}   = figli;
    obstaclesTree.Leaf(i)   = Leaf(i);
    obstaclesTree.limits(:,:,i) = limits;   % limits(1,:) = xrange, limits(2,:) = yrange
    % Sanity check
    if Leaf(i)~=0
        if ~isempty(obstaclesTree.Sons{i})  || sum(sum(objBB(:,:,Leaf(i))-limits))~=0
            error('Only craps, tutto da ricontrollare');
        end
    end
end

obstaclesStruct.obstaclesTree = obstaclesTree;
obstaclesStruct.obstacles = obstacles;
obstaclesStruct.xRange = xRange;
obstaclesStruct.yRange = yRange;

end

