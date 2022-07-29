clc;
clear;
close all
% pcd_file = "F:\Autonomous Robot Lab\Datasets/GlobalMap.pcd";
pcd_file = "GlobalMap.pcd";
ptCloudO = pcread(pcd_file);
gridStep = 0.5;
ptCloud = pcdownsample(ptCloudO,'gridAverage',gridStep);
% figure(1)
% pcshow(ptCloudA);
% roi = [-50,50;-50,50;-inf,inf];
% sampleIndices = findPointsInROI(ptCloudA,roi);
% ptCloud = select(ptCloudA, sampleIndices);
% pcwrite(ptCloud,"pcTest.pcd");

% ptCloud = pcread("pcTest.pcd");
figure(2)
pcshow(ptCloud);

img = ptcloud2map2d(ptCloud,0.5,1,0.3,0.5); %0.2 too slow
% img = edge(img);
% img = img / max(img,[],'all');
% img = img/ 150;
figure(3)
imshow(img)
tmp = split(pcd_file,'/');
tmp = split(tmp{end},'.');
imwrite(img,tmp{1}+".jpg");



% [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
%             maxDistance,referenceVector,maxAngularDistance);
% plane1 = select(ptCloud,inlierIndices);
% remainPtCloud = select(ptCloud,outlierIndices);

% figure(3)
% pcshow(remainPtCloud)
