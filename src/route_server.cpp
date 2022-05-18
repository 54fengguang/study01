#include <memory>
#include <string>
#include <iostream>

#include <grpc++/grpc++.h>
#include <grpc/grpc.h>
#include <grpc++/server.h>
#include <grpc++/server_builder.h>
#include <grpc++/server_context.h>

#include "route.grpc.pb.h"
#include "route_server.h"
#include "road_plan_run_service.h"
#include "common.h"


using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using ::google::protobuf::RepeatedPtrField;


using namespace std;
using namespace haoxing::road;

namespace haoxing{
 namespace road
  {
      RoadPlanServiceImpl::RoadPlanServiceImpl()
      {
        cout <<  "RoadPlanServiceImpl  构造函数调用！！" << endl;
      
      };


      RoadPlanServiceImpl::~RoadPlanServiceImpl()
      {
        cout <<  "RoadPlanServiceImpl  析构函数调用！！" << endl;

      };

      bool RoadPlanServiceImpl::check(const  RoadPlanRequest* request)
      {
        //必填参数是否为空

        //参数格式是否合法

        //参数取值是否在规定范围
        return true;
      };

      
      Status RoadPlanServiceImpl::GetNodeXYZList(ServerContext* context,
                              const  RoadPlanRequest* request, RoadPlanRespond* response)
      {   
        bool checkFlag = check(request);
        if(!checkFlag)
        {
          throw "请求有参数校验不通过！！";
        }

        haoxing::road::RoadPlanRunService  roadPlanRunService;
        GisFieldInfo gisFieldInfo = request->gisfieldinfo();

        //田块参数信息
        vector<PointXYZ> m_FNode;   
        RepeatedPtrField<haoxing::route::NodeXYZ>  nodeXYZs = gisFieldInfo.fieldnodes();

        for(RepeatedPtrField<haoxing::route::NodeXYZ>::iterator itr = nodeXYZs.begin(); itr != nodeXYZs.end();itr ++)
        {
          PointXYZ pointXYZ;
          pointXYZ.x = itr->x();
          pointXYZ.y = itr->y();
          m_FNode.push_back(pointXYZ);
        }

        //障碍物信息
        vector<Obstacle> obstacles; 
        RepeatedPtrField<haoxing::route::ObstacleReq> obstaclereqs =  gisFieldInfo.obstaclereq();
        for (RepeatedPtrField<ObstacleReq>::iterator itr = obstaclereqs.begin();itr != obstaclereqs.end(); itr++)
        {
          Obstacle obstacle;
          obstacle.x =  itr-> x();
          obstacle.y = itr-> y();
          obstacle.z = itr-> z();
          obstacle.r = itr-> r();
          obstacle.r_w = itr-> r_w();
          obstacle.taskIndex = itr-> taskindex();

          obstacles.push_back(obstacle);
        }

        

        //作业规划起始点坐标
        PointXYZ locationNode;     
        NodeXYZ entrance_node = gisFieldInfo.entrance_node();
        locationNode.x = entrance_node.x();
        locationNode.y = entrance_node.y();
        locationNode.z = entrance_node.z();
        locationNode.yaw = entrance_node.yaw();

        //机具参数信息
        Machine machin;  
        FarmTools farmTools = request->farmtools();
        machin.width = farmTools.width();       // 机具幅宽
        machin.lenght = farmTools.length();      // 机具长度
        machin.rightOffset = farmTools.rightoffset(); // 机具右偏移量
        machin.leftOffset = farmTools.leftoffset();  // 机具左偏移量 

        //拖拉机参数信息
        Tractor tra; 
        Vehicle vehicle = request->vehicle();             
        tra.radius = vehicle.radius();               // 最小转弯半径
        tra.lenght = vehicle.length();               // 车与机具整体长度;
        tra.reverse = vehicle.reverse();                // 是否具有倒车功能 -1 无倒车功能, 1 有倒车功能
        tra.overallWidth = vehicle.over_all_width();         // 车体总体宽度
        tra.frontEndtoLocation = vehicle.front_end_to_location();   // 后轮轴到车体前端之间的间距
        tra.rearEndtoLocation = vehicle.rear_end_to_location();    // 后轮轴到车体后端之间的间距(如果加载农机具则为到农机具端之间的距离)
        tra.wheelBase = vehicle.wheel_base();            // 轴距
        tra.type;                  
        tra.locationPoint = -1 ;        // 本体定位点所在位置
        tra.hitchType  = 0;             // 1：半悬挂 0：悬挂 2:机具位于前方
        tra.locaP;

        //路径规划参数信息` 
        PlanParam planParam = request->planparam();
        double interSp = planParam.interspace();               //点间距
        double typeSeeding = planParam.type_seeding();             //播种作业路径选择类型
        double ChangeOperation = planParam.modify_orientation();        //是否调换作业航向
        double AccOverlap = planParam.acc_overlap();              //是否接受路径重覆盖
        double sd = planParam.safe_distance();   //设置高边安全距离值

        roadPlanRunService.running(m_FNode,obstacles,&locationNode,&machin,
                        &tra,interSp,typeSeeding,ChangeOperation,AccOverlap,sd);

        vector<PointXYZ>  path_points = roadPlanRunService.getNode();
        vector<NodeXYZ> road_plan_point_list;
        if(!path_points.empty()){
            cout << "路径规划参数为不空！！" << endl;
            int count = 0;
      
            for (vector<PointXYZ>::iterator iter = path_points.begin(); iter != path_points.end();iter++){
                cout << "第" << count << "个坐标点：" << (*iter).x << "," << (*iter).y ;
                cout << "  速度：" << (*iter).v << "航向角：" << (*iter).yaw << endl;
                count++;

                NodeXYZ nodeXYZ;
                nodeXYZ.set_x((*iter).x);
                nodeXYZ.set_y((*iter).y);
                nodeXYZ.set_z((*iter).z);
                nodeXYZ.set_yaw((*iter).yaw);
                nodeXYZ.set_v((*iter).v);
                nodeXYZ.set_s((*iter).s);
                nodeXYZ.set_curvature((*iter).cur);
                nodeXYZ.set_back((*iter).Back);
                nodeXYZ.set_s((*iter).StrCur);
                nodeXYZ.set_s((*iter).reverse);
                nodeXYZ.set_s((*iter).type);

                road_plan_point_list.push_back(nodeXYZ);
            }
            
        }else{
            cout << "路径规划参数为空！！" << endl;
        }

        response->mutable_nodes()->CopyFrom({road_plan_point_list.begin(),road_plan_point_list.end()});
        response ->set_mileage(100.0);

        return Status::OK;
      };
  } // namespace route  
} // namespace haoxin

