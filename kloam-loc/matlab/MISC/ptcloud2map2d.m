function img = ptcloud2map2d(ptcloud,res,sign,var_thres,groundH)
% res: resolution of the image , in meter
% sign: if inverted,set to -1; if not, 1;
% var_thres: determine if the ground is flat
% groundH: height start from the lowest for ground removal
% 

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
    numP = zeros(xNum,yNum);
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
        z = sign*point(3);
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
            tmp_vec = cellP{i,j}(1:numP(i,j));
            stdT = std(tmp_vec);
            if isempty(tmp_vec)
                img(i,j) = 0;
            elseif stdT < var_thres
                img(i,j) = 0; % transversal area
            else % for indoor
                tmp_vecS = sort(tmp_vec);
                maxH = max(tmp_vecS);
                minH = min(tmp_vecS);
                idx1 = find(tmp_vecS > minH + groundH,1,'first');
                idx2 = find(tmp_vecS > maxH - groundH,1,'first');
                if ~isempty(idx1) && ~isempty(idx2) && idx2>idx1
                    img(i,j) = numP(i,j);
                else
                    img(i,j) = 10;
                end
            end
        end
    end
    maxV = max(img,[],'all')
    img = img/maxV;
    
end