#include "utility.h"
#include "kloam/cloud_info.h"
#include "kloam/save_map.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
// giseop
enum class SCInputType { 
    SINGLE_SCAN_FULL, 
    SINGLE_SCAN_FEAT, 
    MULTI_SCAN_FEAT 
}; 
/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;
typedef geometry_msgs::PoseWithCovarianceStampedConstPtr rvizPoseType;

class mapOptimization : public ParamServer
{

public:
    // for temporal mapping mode
    double rosTimeStart = -1;
    bool temporalMappingMode = false;
    float startTemporalMappingDistThre = 20.0; // key poses sparsified by 2.0 m
    float startTemporalMappingErrorThre = 0.2; // if no serious map out-of-date, it suffice 99% of the time
    float startTemporalMappingInlierRatioThre = 0.2; // even local matching can reach 0.3
    float transformBeforeMapped[6];
    bool goodToMergeMap = false;
    Eigen::Affine3f mergeCorrection;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    // for kitti pose save
    Eigen::Affine3f H_init;
    vector<Eigen::Affine3f> pose_kitti_vec;

    bool doneSavingMap = false;

    Eigen::Affine3f affine_body_lidar;

    vector<double> mapRegistrationError;
    vector<vector<double>> mapErrorPerFrame;
    vector<float> noiseVec;
    pcl::PointCloud<PointType>::Ptr submap;
    vector<rvizPoseType> poseEstVec;
    Eigen::Affine3f relocCorrection;


    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLidarCloudSurround;
    ros::Publisher pubLidarOdometryGlobal;
    ros::Publisher pubLidarOdometryGlobalFusion;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;
    ros::Publisher pubPathFusion;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subLoop;
    ros::Subscriber subGT;
    ros::Subscriber initialpose_sub;
    ros::ServiceServer srvSaveMap;

    std::deque<nav_msgs::Odometry> gpsQueue;
    kloam::cloud_info cloudInfo;
    queue<kloam::cloud_infoConstPtr> cloudInfoBuffer;
    queue<nav_msgs::Odometry::ConstPtr> gtBuffer;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    vector<pcl::PointCloud<PointType>::Ptr> temporalCornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> temporalSurfCloudKeyFrames;
    

    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;
    // pcl::PointCloud<PointType>::Ptr copy2_cloudKeyPoses3D;
    // pcl::PointCloud<PointTypePose>::Ptr copy2_cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr temporalCloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr temporalCloudKeyPoses6D;


    pcl::PointCloud<PointType>::Ptr lidarCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr lidarCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfLastDS; // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr lidarCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> lidarCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> lidarCloudOriCornerFlag;
    std::vector<PointType> lidarCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> lidarCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> lidarCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr lidarCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr lidarCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr lidarCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    
    ros::Time timeLidarInfoStamp;
    double cloudInfoTime;

    float odometryError;
    float transformTobeMapped[6];
    float csmmValue[6];
    float lidarRollInit, lidarPitchInit,lidarYawInit;
    std::mutex mtx;
    std::mutex mtxInit;
    std::mutex mtxLoopInfo;
    std::mutex pose_estimator_mutex;
    // std::mutext mtxReloc;
    bool isDegenerate = false;
    cv::Mat matP;

    int lidarCloudCornerFromMapDSNum = 0;
    int lidarCloudSurfFromMapDSNum = 0;
    int lidarCloudCornerLastDSNum = 0;
    int lidarCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    // map<int, int> loopIndexContainer; // from new to old
    multimap<int,int>    loopIndexContainer;
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    // vector<gtsam::SharedNoiseModel> loopNoiseQueue;
    deque<std_msgs::Float64MultiArray> loopInfoVec;
    vector<nav_msgs::Odometry> globalOdometry;
    nav_msgs::Path globalPath;
    nav_msgs::Path globalPathFusion;

    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;
    Eigen::Affine3f lastFrameMotion;


    float odometryValue[6];
    bool lastLidarPreTransAvailable = false;

    bool poseGuessFromRvizAvailable = false;
    float rvizGuess[6];
    vector<nav_msgs::Odometry> gpsVec;
    
    // Scan context
    std::string saveSCDDirectory;
    pcl::PointCloud<PointType>::Ptr lidarCloudRaw; // giseop
    pcl::PointCloud<PointType>::Ptr lidarCloudRawDS; // giseop
    
    int lastKeyFrameLoopClosed = -1;


    mapOptimization()
    {
        // affine_body_lidar: converts points in lidar frame to body frame, the same convention as in NCLT
        affine_body_lidar = pcl::getTransformation(transBodyLidar[3]*M_PI/180, transBodyLidar[4]*M_PI/180, transBodyLidar[5]*M_PI/180, transBodyLidar[0], transBodyLidar[1], transBodyLidar[2]);
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);
        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("/kloam/mapping/key_poses", 1);
        pubLidarCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("/kloam/mapping/map_global", 1);
        pubLidarOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("/kloam/mapping/odometry", 1);
        pubLidarOdometryGlobalFusion      = nh.advertise<nav_msgs::Odometry> ("/kloam/mapping/odometry_fusion", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("/kloam/mapping/path", 1);
        pubPathFusion               = nh.advertise<nav_msgs::Path>("/kloam/mapping/path_fusion", 1);

        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("/kloam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("/kloam/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/kloam/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("/kloam/mapping/map_local", 1);
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("/kloam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("/kloam/mapping/cloud_registered_raw", 1);

        // subCloud = nh.subscribe<kloam::cloud_info>("/kloam/lidarOdometry/cloud_info_with_guess", 1, &mapOptimization::lidarCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subCloud = nh.subscribe<kloam::cloud_info>("/kloam/feature/cloud_info", 10, &mapOptimization::lidarCloudInfoHandler, this);
        subGT = nh.subscribe<nav_msgs::Odometry> ("/ground_truth", 10, &mapOptimization::gtHandler,this);

        srvSaveMap  = nh.advertiseService("/kloam/save_map", &mapOptimization::saveMapService, this);


        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization
        allocateMemory();
        
    }

    void allocateMemory()
    {
        mergeCorrection = Eigen::Affine3f::Identity();

        submap.reset(new pcl::PointCloud<PointType>()); // why dot when it is pointer type

        lidarCloudRaw.reset(new pcl::PointCloud<PointType>()); // giseop
        lidarCloudRawDS.reset(new pcl::PointCloud<PointType>()); // giseop

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());


        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        // copy2_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        // copy2_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        temporalCloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        temporalCloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        lidarCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        lidarCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        lidarCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        lidarCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        lidarCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        lidarCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        lidarCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        lidarCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        lidarCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(lidarCloudOriCornerFlag.begin(), lidarCloudOriCornerFlag.end(), false);
        std::fill(lidarCloudOriSurfFlag.begin(), lidarCloudOriSurfFlag.end(), false);

        lidarCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        lidarCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        lidarCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        lidarCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformBeforeMapped[i] = 0;
            transformTobeMapped[i] = 0;
            odometryValue[i] = 0;
            csmmValue[i] = 0;
        }
        
        lastFrameMotion = Eigen::Affine3f::Identity();
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void odometryMsgToAffine3f(const nav_msgs::Odometry::ConstPtr& msgIn,Eigen::Affine3f &trans){
        tf::Quaternion tfQ(msgIn->pose.pose.orientation.x,msgIn->pose.pose.orientation.y,msgIn->pose.pose.orientation.z,msgIn->pose.pose.orientation.w);
        double roll,pitch,yaw;
        tf::Matrix3x3(tfQ).getRPY(roll,pitch,yaw);
        
        trans = pcl::getTransformation(msgIn->pose.pose.position.x,
        msgIn->pose.pose.position.y,msgIn->pose.pose.position.z, float(roll),float(pitch),float(yaw));
    }

    void lidarCloudInfoHandler(const kloam::cloud_infoConstPtr& msgIn)
    {
        mtx.lock();
        cloudInfoBuffer.push(msgIn);
        mtx.unlock();
    }

    void gtHandler(const nav_msgs::Odometry::ConstPtr& msgIn)
    {
        // cout<<msgIn->pose.pose.position.x<<endl;
        mtx.lock();
        gtBuffer.push(msgIn);
        mtx.unlock();

        
        Eigen::Affine3f affine_body;
        odometryMsgToAffine3f(msgIn, affine_body); // why not okay when in function?
        Eigen::Affine3f affine_lidar = affine_body*affine_body_lidar;
        float odomTmp[6];
        Affine3f2Trans(affine_lidar,odomTmp);
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(odomTmp[0], odomTmp[1], odomTmp[2]),
                                                      tf::Vector3(odomTmp[3], odomTmp[4], odomTmp[5]));
        // child frame 'lidar_link' expressed in parent_frame 'mapFrame'
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, msgIn->header.stamp, mapFrame, lidarFrame);
        br.sendTransform(trans_odom_to_lidar);

    }

    void run()
    {
        while(ros::ok()){ // why while(1) is not okay???
            while (!cloudInfoBuffer.empty() && !gtBuffer.empty())
            {
                // cout<<cloudInfoBuffer.size()<<" "<<gtBuffer.size()<<endl;
                // normally lidarodometry should be a bit slower than cloudInfoBuffer, if odometry takes more than 100ms
                // so sync based on lidarodometry
                // cloudInfo comes first
                mtx.lock();
                while (!gtBuffer.empty() && gtBuffer.front()->header.stamp.toSec() < cloudInfoBuffer.front()->header.stamp.toSec())
                {
                    gtBuffer.pop();
                }
                if (gtBuffer.empty()){
                    mtx.unlock();
                    break;
                }
                timeLidarInfoStamp = cloudInfoBuffer.front()->header.stamp;
                cloudInfoTime = cloudInfoBuffer.front()->header.stamp.toSec();
                double lidarOdometryTime = gtBuffer.front()->header.stamp.toSec();
                if (rosTimeStart < 0) rosTimeStart = lidarOdometryTime;

                if (fabs(lidarOdometryTime -cloudInfoTime) > 0.05) // normally >, so pop one cloud_info msg
                {
                    ROS_WARN("Unsync message!");
                    cloudInfoBuffer.pop();  // pop the old one,otherwise it  will go to dead loop, different from aloam 
                    mtx.unlock();
                    break;
                }

                // cout<<std::ios::fixed<<setprecision(6)<<lidarOdometryTime<<" "<<cloudInfoTime<<endl;
                // extract info and feature cloud
                TicToc mapping;
                kloam::cloud_infoConstPtr cloudInfoMsg = cloudInfoBuffer.front();
                nav_msgs::Odometry::ConstPtr gtMsg =  gtBuffer.front();

                lidarCloudRaw.reset(new pcl::PointCloud<PointType>()); 
                cloudInfo = *cloudInfoMsg;
                Eigen::Affine3f affine_body;
                odometryMsgToAffine3f(gtMsg,affine_body);
                Eigen::Affine3f affine_lidar = affine_body*affine_body_lidar;
                Affine3f2Trans(affine_lidar,transformTobeMapped);
                
                // cout<<"original quaternion: "<<gtMsg->pose.pose.orientation.x<<" "<<gtMsg->pose.pose.orientation.y<<
                //     " "<<gtMsg->pose.pose.orientation.z<<" "<<gtMsg->pose.pose.orientation.w<<endl;         

                pcl::fromROSMsg(cloudInfoMsg->cloud_corner,  *lidarCloudCornerLast);
                pcl::fromROSMsg(cloudInfoMsg->cloud_surface, *lidarCloudSurfLast);
                pcl::fromROSMsg(cloudInfoMsg->cloud_raw,  *lidarCloudRaw);

                if (lidarCloudCornerLast->points.empty() || lidarCloudSurfLast->points.empty() ||
                     lidarCloudCornerLast->size() < 10 || lidarCloudSurfLast->size() < 100)
                {
                    ROS_WARN("map optimization:  an empty or degraded frame ");
                    gtBuffer.pop(); 
                    while (!cloudInfoBuffer.empty())
                    {
                    cloudInfoBuffer.pop();
                    // ROS_INFO_STREAM("popping old cloud_info messages for real-time performance");
                    }
                    mtx.unlock();
                    return; 
                }

                // clear
                gtBuffer.pop(); 
                while (!cloudInfoBuffer.empty())
                {
                    cloudInfoBuffer.pop();
                    // ROS_INFO_STREAM("popping old cloud_info messages for real-time performance");
                }
                mtx.unlock();

                saveKeyFramesAndFactor();

                publishFrames();
                
                publishOdometry();
                
                ROS_INFO_STREAM("Mapping takes "<<mapping.toc()<<"ms");
                
            }


        }

    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        // #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }


    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    
    bool saveMapService(kloam::save_mapRequest& req, kloam::save_mapResponse& res)
    {
    //   string saveMapDirectory=req.destination;
        if (savePCD){
            cout << "****************************************************" << endl;
            cout << "Saving map to pcd files ..." << endl;
            cout << "Save destination: " << saveMapDirectory << endl;
            // save key frame transformations
            pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);
            // extract global point cloud map
            pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
            for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
                cout<<cornerCloudKeyFrames[i]->size()<<endl;
                cout<<cloudKeyPoses6D->points[i].x<<endl;
                *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
                *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
            }

            if(req.resolution != 0)
            {
                cout << "\n\nSave resolution: " << req.resolution << endl;

                // down-sample and save corner cloud
                downSizeFilterCorner.setInputCloud(globalCornerCloud);
                downSizeFilterCorner.setLeafSize(req.resolution, req.resolution, req.resolution);
                downSizeFilterCorner.filter(*globalCornerCloudDS);
                pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);
                // down-sample and save surf cloud
                downSizeFilterSurf.setInputCloud(globalSurfCloud);
                downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
                downSizeFilterSurf.filter(*globalSurfCloudDS);
                pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
            }
            else
            {
                // save corner cloud
                pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloud);
                // save surf cloud
                pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);
            }

            // save global point cloud map
            *globalMapCloud += *globalCornerCloud;
            *globalMapCloud += *globalSurfCloud;

            int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
            res.success = ret == 0;

            // downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
            // downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

            cout << "Saving map to pcd files completed\n" << endl;
        }
          // saving pose estimates and GPS signals
        if (savePose)
        {
            ofstream pose_file;
            cout<<"Recording trajectory..."<<endl;
            string fileName = saveMapDirectory+"/pose_kloam.txt";
            pose_file.open(fileName,ios::out);
            pose_file.setf(std::ios::scientific, std::ios::floatfield);
            pose_file.precision(6);
            if(!pose_file.is_open())
            {
                cout<<"Cannot open "<<fileName<<endl;
                return false;
            }
            // keyposes: kitti form (z forward x right y downward)
            cout<<"Number of poses: "<<pose_kitti_vec.size()<<endl;
            for (int i = 0; i <(int)pose_kitti_vec.size(); ++i)
            {
                Eigen::Affine3f pose_kitti = pose_kitti_vec[i];
                pose_file<<pose_kitti(0,0)<<" "<<pose_kitti(0,1)<<" "<<pose_kitti(0,2)<<" "<<pose_kitti(0,3)<<" "
                    <<pose_kitti(1,0)<<" "<<pose_kitti(1,1)<<" "<<pose_kitti(1,2)<<" "<<pose_kitti(1,3)<<" "
                    <<pose_kitti(2,0)<<" "<<pose_kitti(2,1)<<" "<<pose_kitti(2,2)<<" "<<pose_kitti(2,3)<<"\n";

            }
            pose_file.close();


            // // 2nd: keyposes
            // int pointN = (int)globalPath.poses.size();
            // cout<< "There are "<<pointN<<" keyframes in total"<<endl;
            // for (int i = 0; i < pointN; ++i)
            // {
            //     geometry_msgs::PoseStamped tmp = globalPath.poses[i];
            //     pose_file<<tmp.pose.position.x<<" "<<tmp.pose.position.y<<" "<<tmp.pose.position.z<<"\n";
            // }
            // // 3rd: odometry msgs
            // int pointN = (int)globalOdometry.size();
            // cout<< "There are "<<pointN<<" in total"<<endl;
            // for (int i = 0; i < pointN; ++i)
            // {
            //     nav_msgs::Odometry tmp = globalOdometry[i];
            //     pose_file<<tmp.pose.pose.position.x<<" "<<tmp.pose.pose.position.y<<"\n";
            // }
            // pose_file.close();

            // 4th: stamped pose for odometry gt
            ofstream pose_file2;

            string fileName2 = saveMapDirectory+"/path_6DOF.txt";
            pose_file2.open(fileName2,ios::out);
            pose_file2.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数
            pose_file2.precision(6); // 固定小数位6
            // int pointN = (int)globalOdometry.size();
            // for (int i = 0; i < pointN; ++i)
            // {
            //     nav_msgs::Odometry tmp = globalOdometry[i];
            //     double r,p,y;
            //     tf::Quaternion q(tmp.pose.pose.orientation.x,tmp.pose.pose.orientation.y, tmp.pose.pose.orientation.z,tmp.pose.pose.orientation.w);
            //     tf::Matrix3x3(q).getRPY(r,p,y);
            //     // save it in nano sec to compare it with the nclt gt
            //     pose_file2<<tmp.header.stamp.toSec()*1e+6<<" "<<r<<" "<<p<<" "<<y<<" "<<tmp.pose.pose.position.x<<" "<<tmp.pose.pose.position.y<<" "<<
            //     tmp.pose.pose.position.z<<"\n";
            // }
            // pose_file2.close();
            // cout<<"Trajectory recording finished!"<<endl;

            // .size() returns unsigned size_type, if vec is empty, 0-1 would sth very big
            int pointN = cloudKeyPoses6D->size();
            cout<<"Number of key poses: "<< pointN<<endl;
            for (int i = 0; i < pointN; ++i)
            {
                pose_file2<<cloudKeyPoses6D->points[i].x<<" "<<cloudKeyPoses6D->points[i].y<<" "<<cloudKeyPoses6D->points[i].z
                <<" "<<cloudKeyPoses6D->points[i].roll<<" "<<cloudKeyPoses6D->points[i].pitch<<" "<<cloudKeyPoses6D->points[i].yaw<<" "<<i<<"\n";

            }
            pose_file2.close();
            cout<<"Trajectory recording finished!"<<endl;
        }
    //   if (savePose){
    //     ofstream gps_file;
    //     gps_file.open(saveMapDirectory+"/gps.txt",ios::out);
    //     if(!gps_file.is_open()){
    //         cout<<"Cannot open"<<saveMapDirectory+"/gps.txt"<<endl;
    //     }

    //     for (int i = 0; i <int(gpsVec.size()); ++i)
    //     {
    //         gps_file<<gpsVec[i].pose.pose.position.x<<" "<<gpsVec[i].pose.pose.position.y<<" "<<gpsVec[i].pose.pose.position.z<<"\n";
    //     }
    //     gps_file.close();
    //     cout<< "GPS data were stored!"<<cloudKeyPoses6D->size()<<" "<<gpsVec.size()<<endl;
    //   }

        // save keyframe map: for every keyframe, save keyframe pose, edge point pcd, surface point pcd
        // keyframe pose in one file
        // every keyframe has two other files: cornerI.pcd surfI.pcd
        if(saveKeyframeMap){ 
            int keyframeN = (int)cloudKeyPoses6D->size();
            // ros is already shut down, don't use ROS_INFO!
            cout<<"Map Saving to "+saveKeyframeMapDirectory<<endl;
            cout<<"There are "<<keyframeN<<" keyframes before downsampling"<<endl;

            cout<<"********************Saving keyframes and poses one by one**************************"<<endl;
            pcl::PointCloud<PointType>::Ptr cloudKeyPoses3DDS(new pcl::PointCloud<PointType>());
            downSizeFilterSurroundingKeyPoses.setInputCloud(cloudKeyPoses3D); // 2.0m
            downSizeFilterSurroundingKeyPoses.filter(*cloudKeyPoses3DDS);
            int keyframeNDS = cloudKeyPoses3DDS->size();
            cout<<"There are "<<keyframeNDS<<" keyframes after downsampling"<<endl;
            ofstream pose_file;
            pose_file.open(saveKeyframeMapDirectory+"/poses.txt",ios::out); // downsampled
            if(!pose_file.is_open()){
                std::cout<<"Cannot open"<<saveKeyframeMapDirectory+"/poses.txt"<<std::endl;
                return false;
            }
            std::vector<int> keyframeSearchIdx;
            std::vector<float> keyframeSearchDist;
            pcl::KdTreeFLANN<PointType>::Ptr kdtreeKeyframes(new pcl::KdTreeFLANN<PointType>());
            kdtreeKeyframes->setInputCloud(cloudKeyPoses3D);
            int i = 0;
            for(auto& pt:cloudKeyPoses3DDS->points)
            {                
                kdtreeKeyframes->nearestKSearch(pt,1,keyframeSearchIdx,keyframeSearchDist); 
                pt.intensity = cloudKeyPoses6D->points[keyframeSearchIdx[0]].intensity;  
                pcl::io::savePCDFileBinary(saveKeyframeMapDirectory + "/corner" + std::to_string(i)+".pcd", *cornerCloudKeyFrames[pt.intensity]);
                pcl::io::savePCDFileBinary(saveKeyframeMapDirectory + "/surf" + std::to_string(i)+".pcd", *surfCloudKeyFrames[pt.intensity]);
                // here we need to re-arrange the indexes!
                pose_file<<cloudKeyPoses6D->points[pt.intensity].x<<" "<<cloudKeyPoses6D->points[pt.intensity].y<<" "<<cloudKeyPoses6D->points[pt.intensity].z
                <<" "<<cloudKeyPoses6D->points[pt.intensity].roll<<" "<<cloudKeyPoses6D->points[pt.intensity].pitch<<" "<<cloudKeyPoses6D->points[pt.intensity].yaw
                << " " << i<<"\n";
                i++;
                if((i+1)%100 == 0) cout<<i<<" keyframes saved!"<<endl;
            }
            cout<<"keyframes Saving Finished!"<<endl;

            // .size() returns unsigned size_type, if vec is empty, 0-1 would be very big
            pose_file.close();
            // cout<<"********************Saving Scan Context one by one**************************"<<endl;
            // scManager.saveScanContext(saveKeyframeMapDirectory+"/scanContext");
        }
        res.success = true;
        doneSavingMap = true;
        return true;
    }

    // void generateOGthread()
    // {

    // }

    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        // ROS_INFO("start publishing global maps");
        while (ros::ok()){
            // publish key poses
            
            publishCloud(&pubKeyPoses, cloudKeyPoses3D, timeLidarInfoStamp, mapFrame);
            
            publishGlobalMap();
            // ROS_INFO("publishing global maps");
            rate.sleep();
        }

        if (savePCD == false && saveKeyframeMap == false && savePose == false)
            return;

        kloam::save_mapRequest  req;
        kloam::save_mapResponse res;

        if (!doneSavingMap)
        {
            if(!saveMapService(req, res))   cout << "Fail to save map" << endl;
        }
    }

    void publishGlobalMap()
    {
        if (pubLidarCloudSurround.getNumSubscribers() == 0)
        {
            return;
        }
            

        if (cloudKeyPoses3D->points.empty() == true)
        {
            // ROS_INFO("No key poses!");
            return;
        }
        // cout<<"2"<<endl;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize

        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key poses 
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        // fix the keyframe downsample bug
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        // only for visualization
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
        {
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

        

        publishCloud(&pubLidarCloudSurround, globalMapKeyFramesDS, timeLidarInfoStamp, mapFrame);

        // publish history key_poses

    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty()) // 第一帧为关键帧
            return true;
        // allow overlapped area to display loop closures
        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)

            return false;
        // cout<<x<<" "<<y<<" "<<z<<endl;
        // cout<<roll<<" "<<pitch<<" "<<yaw<<endl;
        return true;
    }


    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        thisPose3D.x = transformTobeMapped[3];
        thisPose3D.y = transformTobeMapped[4];
        thisPose3D.z = transformTobeMapped[5];
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as keyframe index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = transformTobeMapped[0];
        thisPose6D.pitch = transformTobeMapped[1];
        thisPose6D.yaw   = transformTobeMapped[2];
        thisPose6D.time = cloudInfoTime;
        cloudKeyPoses6D->push_back(thisPose6D);

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());

        pcl::copyPointCloud(*lidarCloudCornerLast,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*lidarCloudSurfLast,    *thisSurfKeyFrame);
        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame); // 这个全局都存着，但每次局部匹配只搜索50m内的关键帧
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);
        



    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry lidarOdometryROS;
        lidarOdometryROS.header.stamp = timeLidarInfoStamp;
        lidarOdometryROS.header.frame_id = mapFrame;
        lidarOdometryROS.child_frame_id = odometryFrame;
        lidarOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        lidarOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        lidarOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        lidarOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLidarOdometryGlobal.publish(lidarOdometryROS);
        globalOdometry.push_back(lidarOdometryROS);
    }

    void publishFrames()
    {
        globalPath.header.stamp = timeLidarInfoStamp;
        globalPath.header.frame_id = mapFrame;
        pubPath.publish(globalPath);
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kloam");

    mapOptimization MO_ground_truth;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO_ground_truth);

    std::thread mappingThread{&mapOptimization::run,&MO_ground_truth};

    ros::spin();

    visualizeMapThread.join();
    mappingThread.join();

    return 0;
}
