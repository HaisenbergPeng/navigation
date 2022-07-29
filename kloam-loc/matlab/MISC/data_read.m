clc;
clear;
close all
% %% gps数据并非定频率采样 requires conversion from lat,longi to x y
% gps=csvread('/media/binpeng/BIGLUCK/Datasets/NCLT/datasets/2012-01-08/20120108/gps.csv');
% gps_rtk=csvread('/media/binpeng/BIGLUCK/Datasets/NCLT/datasets/2012-01-08/20120108/gps_rtk.csv');
% % gps_fre=1e+6/(gps(2,1)-gps(1,1));
% % gps_rtk_fre=1e+6/(gps_rtk(2,1)-gps_rtk(1,1));
% figure(1)
% plot(gps(:,4),gps(:,5))
% hold on
% plot(gps_rtk(:,4),gps_rtk(:,5))
% legend('gps','gps_rtk');
% figure(2)
% plot(gps(:,1))
% hold on
% plot(gps_rtk(:,1));
% %% imu九轴数据:精确到100ns，加速度+角速度与欧拉角基本是同步的
% ms25=csvread('ms25.csv');
% ms25E=csvread('ms25_euler.csv');
% figure(1)
% plot(ms25(:,1))
% hold on
% plot(ms25E(:,1));

% %% ground truth 
% dates = ["2012-01-08","2012-01-15","2012-01-22","2012-02-02","2012-02-12","2012-05-11"];
% for i = 1:length(dates)
%     filename = "/media/binpeng/BIGLUCK/Datasets/NCLT/datasets/groundtruth_"+dates(i)+".csv";
%     data=csvread(filename);
%     n = length(data(:,1));
%     figure(1)
%     p = plot(data(:,2),data(:,3),'LineWidth',1);
% %     % modified jet-colormap // not working!
% %     cd = [uint8(jet(n)*255) uint8(ones(n,1))];  
% %     set(p.Edge, 'ColorBinding','interpolated', 'ColorData',cd) 
%     hold on
% end
% legend(dates);

%% pcd read
pcd_name = '/media/binpeng/BIGLUCK/Datasets/NCLT/datasets/2012-01-08/KLOAM/map_pcd/GlobalMap.pcd';
pc = pcread(pcd_name);
pcshow(pc);