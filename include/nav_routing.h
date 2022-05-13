/**
 * @copyright Copyright (c) 2020 Bright Dream Robotics Co.Ltd
 * @file nav_routing.h
 * @brief 
 * @author tanglingmao (tanglingmao@countrygardent.com.cn)
 * @date 2020-06-30
 * @version 0.1.0
 */
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <fstream>
#include <cstdlib>
#include "std_msgs/Int8.h"
#include <limits>
#include <ctime>
#include <unistd.h> 


#include "std_msgs/String.h"
#include "routing.h"

// #include "ar_msgs/chassis_control.h"
// #include "ar_msgs/base_line.h"
// #include "ar_msgs/control.h"
// #include "ar_msgs/chassis.h"
#include "std_msgs/Float32.h"
// #include "ar_common/log.h"
// #include "path.pb.h"
// #include "ar_msgs/local_trajectory.h"
// #include "ar_msgs/trajectory.h"
// #include "ar_msgs/planning.h"
// #include "ar_topic/common_topic.h"
//#include "ar_msgs/localization.h"

#include "common.h"
// #include "smooth.h"
#include "segmentlane.h"
#include "fishtail.h"
#include "pearshape.h"
#include "directseeding.h"
#include "publishpath.h"
#include "harv.h"
#include <thread>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
using namespace std;
//using namespace DubinsPaths;
class nav_routing
{
private:
    ros::NodeHandle m_nh;
    ros::NodeHandle nh_private;
    ros::Publisher m_pubPath;
    ros::Publisher m_testPubPath;
    ros::Publisher m_testPubAb;
    ros::Publisher m_pubDisplayPath;
    ros::Publisher m_pubAlarmInfo;
    ros::Publisher m_pathProtoPub;
    ros::Publisher m_pubBaseLineState;
    ros::Publisher m_pubTranslation;
    ros::Publisher m_pubTransfer;
    ros::Publisher m_pub;
    ros::Publisher m_pubCableComVal;

    ros::Subscriber m_gnssCurrentPose;
    ros::Subscriber m_transferSub;
    ros::Subscriber m_routingBasicInfo;
    ros::Subscriber m_baseLineTranslation;
    ros::Subscriber m_controlCmd;
    ros::Subscriber m_vec;
    ros::Subscriber m_Chassisresponse;
    ros::Subscriber m_ControlBaseInfo;
    ros::Subscriber m_action;
    ros::Subscriber m_backInfo;
    ros::Subscriber m_Cable;

    

    nav_msgs::Path m_path;
    nav_msgs::Path m_testPath;
    nav_msgs::Path m_displayPath;
    // ar_msgs::base_line m_baseLineInfo;
    // ar_msgs::chassis_control m_ChassisControl;
    // ar_msgs::trajectory m_trajectory;
    // ar_msgs::trajectory m_disptrajectory;
    // ar_msgs::local_trajectory m_lt;
    std_msgs::Int8  CableVal;


    // bgy::routing::RoutingAB m_rAB;
    bgy::routing::RoutingABCD m_rABCD;
    bgy::common::math::Math m_math;
    bgy::common::Common common;
    fishtail ft;
    // bgy::routing::harvesting har;
    bgy::routing::pear pe;
    // bgy::routing::ploughing pl;
    bgy::routing::directseeding::water dsw;
    bgy::routing::harvest har;
    SegLane SL;
    // mmc MC;
    // bgy::smooth m_smooth;

    // roadopt RO;


    publishpath PPath;

    Tractor m_tractor;
    Machine m_machine;
    SystemState m_systemState;
    ShiftDatum m_shiftInfo;
    FarmImplementOpDis FIO;

    V2P m_fieldLocation;
    V2P m_poseInfo;
    V2P m_tempPoseInfo;
    V2P m_pubPathInfo;
    V2P m_tpath;
    
    V2P m_NewPub;
    V2P m_hisPoint;
    PointXYZ m_tempp;
    V2P m_refLane;
    float UTNL; // update to next lane
    int m_typeSeeding;
    bool m_ChangeOperation; // 修改作业方向
    bool m_AcceptOverlap;
    

    
    double m_interpRes; /// 参考直线点上的插值间距大小
    PointXYZ m_locationPoint;
    double m_originX;
    double m_originY;
    double m_tK = 0.0;
    double m_downVal = 5.0;
    double m_UporDown = 4;
    float m_Down = 3.0;
    double m_safeDistance;
    int m_reset;
    int size_his = 0;
    int run_display = 0;
    int m_StartType;   //启动类型: 1:launch 2:平台端
    
    int8_t m_flag = 1; /// 用于监测routing运行条件是否满足
    int16_t m_routeType = 0;
    vector<V2P> m_newSegmentLanePoint;
    vector<V2P> m_hisSegmentLanePoint;
    std::vector<V2P> m_ttPath;
    int m_testInfo = 0;
    int m_tChassisstate = 3;
    int m_tChassissPtoState = -1;
    int8_t m_tChassisresponse = 0;

    float route_length = 20; //定义路径长度为20m
    float m_extlength = 20;

    int runtime = 0;
    int m_Onlinestate = 0;
    int m_hisCh = -1 ;
    int m_pubofLane = 0;
    int fishlab = 0;
    int lab1 = 0;
    int lab2 = 0;
    int m_buildtree = 0;
    int recPose = 0;
    bool run_Finish = false;
    bool m_EdgePlanning;


    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLane;



    std::thread m_pubThread;

    void *pubThreadEntry();
    bool m_runningflag;
    float m_Vec;
    bool m_driveState = false;
    
    std::vector<int> CableComVal; // 第一位1 放 2 收, 第二位 0 不改变当前状态01 收或者放, 2 停;
    int hisCableComVal; // 1 放 2 收;

    V2P point1;
    V2P point2;
    V2P point;

    V2P testFish;

    PointXYZ tpp1;
    PointXYZ tpp2;

    double road_line_width = 3.0;

public:

    //fishtail ft;

    nav_routing();
    ~nav_routing();

    void initValue();
    void init(ros::NodeHandle &m_nh);

    /**
     * @brief pose 信息回调函数 
     * @param[in] msg 
     */
    void onPoseReceived(const nav_msgs::Odometry::ConstPtr &msg);

    /**
     * @brief 规划基础信息回调函数
     * @param[in] msg 
     */
    // void onBaseLineRoutingReceived(const ar_msgs::base_line::ConstPtr &msg);

    /**
     * @brief 机具偏差量回调函数
     * @param[in] msg 
     */
   // void onTransferLineReceived(const bgy_ar_msg::transfer_line::ConstPtr &msg);

    /**
     * @brief 基准线偏移回调函数
     * @param[in] msg 
     */
   // void onBaseLineTransReceived(const bgy_ar_msg::base_line_translation::ConstPtr &msg);

    /**
     * @brief 系统控制姿态回调函数
     * @param[in] msg 
     */
    //void onControlCmdReceived(const bgy_ar_msg::control_cmd::ConstPtr &msg);

    // void onCanbusBasicInfoReceived(const ar_msgs::chassis::ConstPtr &msg);

    SystemState getSysteminfo();

    /**
     * @brief 发布规划路径信息
     */
    void publisherofpath();
    /**
     * @brief 显示车辆所走过路径，测试使用
     */
    void publishInittp();

    void runRoutingPath();

    int pathProtoPub();

    void PathPub();
    void run();
    
    void Delay(int time);

        //testing

     /**
     * @brief Get the Path Point object
     * @param[in] p 
     * @return geometry_msgs::PoseStamped 
     */
    geometry_msgs::PoseStamped getPathPoint(vector<double>  p);

    /**
     * @brief 用于测试发点
     */
    void pubTest(V2P p);
    //void pubTest(PointXYZ p);
    void pubTest(std::vector<V2P> p);
    /**
     * @brief 
     * @param[in] p 
     * @return 
     */
    void calcSegmentLane(V2P const& p);

    void onVecReceived(const std_msgs::Float32::ConstPtr &msg);

    // void onControlBasicInfoReceived(const ar_msgs::control::ConstPtr &msg);

    void onActionReceived(const std_msgs::Int8 &msg);

    void calcSegmentLane1(V2P const &p);

    void readRefLane();

    void onCableReceived(const std_msgs::Int8 &msg);

    void pubComV();
    void pubTest1();
    /**
     * @brief 读入GIS数据
     */
    void readGISData();

    
};

//#endif