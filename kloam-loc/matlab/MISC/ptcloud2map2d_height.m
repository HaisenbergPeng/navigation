function img = ptcloud2map2d(ptcloud, height,res, xLen, yLen)
    tol = 0.1;
    num_points = ptcloud.Count;
    xNum = ceil(xLen/res);
    yNum = ceil(yLen/res);
    img = zeros(xNum,yNum);
    %% just extract point clouds with height in [height, height+0.3] 
    for i =1:num_points
        point = ptcloud.Location(i,:);
        x = point(1) + 50;
        y = point(2) + 50;
        if point(3) > height -tol && point(3) < height+ tol
            img(floor(x/res)+1,floor(y/res)+1) = 1;
        end
    end
    
end