function img = ptcloud2map2d(ptcloud,res,var_thres)
% res: resolution of the image , in meter
% sign: if inverted,set to -1; if not, 1;
% comp_len: comparative length, in meter, for normalizing the max height

% image coordinates are in the lidar frame
    num_points = ptcloud.Count;
    x2 = ptcloud.XLimits(2);
    x1 = ptcloud.XLimits(1);
    y2 = ptcloud.YLimits(2);
    y1 = ptcloud.YLimits(1);
    xNum = ceil( (x2 - x1)/res);
    yNum = ceil((y2 - y1)/res);
    img = zeros(xNum,yNum);
    cellP = cell(xNum,yNum);
    numP = cell(xNum,yNum);
    enough_big = 1000;
    for i=1:xNum
        for j=1:yNum
            tmp=zeros(enough_big,1);
            cellP{i,j} = tmp;
        end
    end
    for i =1:num_points
        point = ptcloud.Location(i,:);
        x = point(1);
        y = point(2);
        z = point(3);
        %% ground removal: height variation
        tmp = numP(floor((x-x1)/res) + 1, floor((y-y1)/res) + 1);
        numP(floor((x-x1)/res) + 1, floor((y-y1)/res) + 1) =...
            tmp+1;
        if tmp > enough_big
            disp("enough_big is not big enough");
            break;
        end
        cellP{floor((x-x1)/res) + 1, floor((y-y1)/res) + 1}...
            (numP(floor((x-x1)/res) + 1, floor((y-y1)/res) + 1)) = z; 
    end
    
    for i=1:xNum
        for j=1:yNum
            cellP{i,j} = tmp;
        end
    end
    % max height: relative to the average of the surrouding environment
    % to account for the terrain change: NOT Continuous
%     xNum2 = ceil( (x2 - x1)/comp_len);
%     yNum2 = ceil((y2 - y1)/comp_len);
%     for i = 1:xNum2
%         for j = 1:yNum2
%             x_start = ( i - 1 )*comp_len + 1;
%             x_end = i*comp_len;
%             y_start = ( j - 1 )* comp_len + 1;
%             y_end = j*comp_len;
% %             meanH = mean(img( x_start:x_end ,y_start:y_end ),'all');
%             maxH = max(img( x_start:x_end ,y_start:y_end ),[],'all');
%             minH = min(img( x_start:x_end ,y_start:y_end ),[],'all');
%             img( x_start:x_end ,y_start:y_end ) = (img( x_start:x_end ,y_start:y_end )-minH)/(maxH - minH);
%         end
%     end
end