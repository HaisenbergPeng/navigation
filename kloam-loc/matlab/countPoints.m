function [img,originX,originY] = countPoints(ptcloud,res,gray_thres)
% res: resolution of the image , in meter
% sign: if inverted,set to -1; if not, 1;
% var_thres: determine if the ground is flat
% groundH: height start from the lowest for ground removal
% 
% image coordinates are in the lidar frame
    num_points = ptcloud.Count;
    x2 = ptcloud.XLimits(2);
    x1 = ptcloud.XLimits(1);
    y1 = -ptcloud.YLimits(2);
    y2 = -ptcloud.YLimits(1);
    xNum = ceil( (x2 - x1)/res);
    yNum = ceil((y2 - y1)/res);
    originX = x1;
    originY = -y2;
    
    enough_big = 1e+6;
    gridZMin = enough_big*ones(yNum,xNum);
    numP = zeros(yNum,xNum);

    for i =1:num_points
        point = ptcloud.Location(i,:);
        x = point(1);
        y = -point(2);
        z = point(3);
        
        pixelX = floor((x-x1)/res) + 1;
        pixelY = floor((y-y1)/res) + 1;
        if z < gridZMin(pixelY,pixelX)  % matlab img Y is the 1st axis
            gridZMin(pixelY,pixelX) = z;
        end
        if z < gridZMin(pixelY,pixelX)+2.0 % ruling out ceiling
            numP(pixelY,pixelX) = numP(pixelY,pixelX)+1;
        end

    end
    
    maxV = max(numP,[],'all');
    img = 1- numP/maxV;
    
    for i=1:xNum
        for j=1:yNum
            if img(j,i) > gray_thres
                img(j,i) = 1;
            else
                img(j,i) = 0;
            end
        end
    end
    
end