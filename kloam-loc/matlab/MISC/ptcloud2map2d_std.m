function img = ptcloud2map2d_std(ptcloud,res,sign)
    num_points = ptcloud.Count;
    x2 = ptcloud.XLimits(2);
    x1 = ptcloud.XLimits(1);
    y2 = ptcloud.YLimits(2);
    y1 = ptcloud.YLimits(1);
    xNum = ceil( (x2 - x1)/res);
    yNum = ceil((y2 - y1)/res);
    img = zeros(xNum,yNum);
    
    
    %% just extract point clouds with height in [height, height+0.3] 
    for i =1:num_points
        point = ptcloud.Location(i,:);
        x = point(1);
        y = point(2);
        z = sign*point(3);
%         % accumulate
%         img(floor((x-x1)/res) + 1, floor((y-y1)/res) + 1) = img(floor((x-x1)/res) + 1, floor((y-y1)/res) + 1) + 1;
        % max height
        if img(floor((x-x1)/res) + 1, floor((y-y1)/res) + 1) < z 
            img(floor((x-x1)/res) + 1, floor((y-y1)/res) + 1)  = z;
        end
        
    end
end