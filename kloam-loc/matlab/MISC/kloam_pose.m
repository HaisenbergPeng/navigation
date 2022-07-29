clc;
clear;
close all

%% ground truth 
dates = ["2012-01-08"];
figure(1)
for i = 1:length(dates)
    filename = "/media/binpeng/BIGLUCK/Datasets/NCLT/datasets/groundtruth_"+dates(i)+".csv";
    data=csvread(filename);
    n = length(data(:,1));
    figure(1)
    p = plot(data(:,2),data(:,3),'LineWidth',1);
%     % modified jet-colormap // not working!
%     cd = [uint8(jet(n)*255) uint8(ones(n,1))];  
%     set(p.Edge, 'ColorBinding','interpolated', 'ColorData',cd) 
    hold on
end


% file_kloam = "/media/binpeng/BIGLUCK/Datasets/NCLT/test/keyframes/poses.txt";
file_kloam = "/media/binpeng/BIGLUCK/Datasets/NCLT/datasets/2012-01-08/KLOAM/keyframes/poses.txt";
f = fopen(file_kloam);
poses = textscan(f,"%f%f%f%f%f%f%f");
plot(poses{1},poses{2},'.');


% file_kloam = "/media/binpeng/BIGLUCK/Datasets/NCLT/datasets/2012-01-15/loc_test2/path_6DOF.txt";
file_kloam = "/media/binpeng/BIGLUCK/Datasets/NCLT/datasets/2012-01-08/algo_test/map_pcd/path_6DOF.txt";
f = fopen(file_kloam);
poses2 = textscan(f,"%f%f%f%f%f%f%f");
plot(poses2{5}+11,poses2{6}-182.89,'.');
legend("gt","kloam map","kloam loc");
% legend("gt","kloam loc");

figure(2)
histogram(data(:,4))
figure(3)
histogram(poses{3})