#include <iostream>
#include <memory>
#include <string>
 
#include <grpc++/grpc++.h>
#include <grpc/support/log.h>
 
#include "route.grpc.pb.h"
#include "route_client.h"
 
using grpc::Channel;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;

using namespace haoxing::road;
using namespace haoxing::route;
 
int main(int argc, char** argv) {
  RoadPlanClient client(grpc::CreateChannel(
      "localhost:50051", grpc::InsecureChannelCredentials()));

    //田快参数信息 
    GisFieldInfo gisFieldInfo;

    //田块坐标信息
    NodeXYZ *pointXYZ_1 = gisFieldInfo.add_fieldnodes();
    pointXYZ_1->set_x(691351.408912731);
    pointXYZ_1->set_y(2605105.082928548); 


    NodeXYZ *pointXYZ_2 = gisFieldInfo.add_fieldnodes();
    pointXYZ_2->set_x(691243.873891062);
    pointXYZ_2->set_y(2604957.124816442);

    NodeXYZ *pointXYZ_3 = gisFieldInfo.add_fieldnodes();
    pointXYZ_3->set_x(691189.190118195);
    pointXYZ_3->set_y(2604998.660585673);

    NodeXYZ *pointXYZ_4 = gisFieldInfo.add_fieldnodes();
    pointXYZ_4->set_x(691295.904201433);
    pointXYZ_4->set_y(2605146.617783679);

    gisFieldInfo.add_preset_spaces(0.5);
    gisFieldInfo.add_preset_spaces(0.5);
    gisFieldInfo.add_preset_spaces(0.5);
    gisFieldInfo.add_preset_spaces(0.5);

    //障碍物信息

    //作业规划起始点坐标
    NodeXYZ locationNode;
    locationNode.set_x(691245.118633283);
    locationNode.set_y(2604958.837441858);
    gisFieldInfo.mutable_entrance_node()->MergeFrom(locationNode);


    //机具参数信息
    FarmTools farmTool;  
    farmTool.set_width(2.3);       // 机具幅宽
    farmTool.set_length(1.0);     // 机具长度
    farmTool.set_rightoffset(0.001); // 机具右偏移量
    farmTool.set_leftoffset(0.001);  // 机具左偏移量 

    
    //车辆参数信息
    Vehicle vehicle;              
    vehicle.set_radius(7.0);                   // 最小转弯半径
    vehicle.set_length(4.2);                  // 车与机具整体长度;
    vehicle.set_reverse(1);                   // 是否具有倒车功能 -1 无倒车功能, 1 有倒车功能
    vehicle.set_over_all_width(6.0);          // 车体总体宽度
    vehicle.set_front_end_to_location(1.7);   // 后轮轴到车体前端之间的间距
    vehicle.set_rear_end_to_location(1.7);    // 后轮轴到车体后端之间的间距(如果加载农机具则为到农机具端之间的距离)
    vehicle.set_wheel_base(2.4);              // 轴距


    //作业参数规划
    PlanParam planParam;
    planParam.set_interspace(0.1);      //点间距
    planParam.set_type_seeding(1);        //播种作业路径选择类型
    planParam.set_modify_orientation(-1); //是否调换作业航向
    planParam.set_acc_overlap(1);      //是否接受路径重覆盖
    planParam.set_safe_distance(0.5); //设置高边安全距离值

    vector<haoxing::route::NodeXYZ> path_points;
    try
    {
        bool flag = client.getNodeXYZList(gisFieldInfo,farmTool,vehicle,planParam,path_points);    
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }

    if(!path_points.empty()){
        cout << "路径规划参数为不空！！" << endl;
        int count = 0;
        cout.precision(12);
        for (vector<NodeXYZ>::iterator iter = path_points.begin(); iter != path_points.end();iter++){
            cout << "第" << count << "个坐标点：" << iter->x() << "," << iter->y() ;
            cout << "  速度：" << iter->v() << "航向角：" << iter->yaw() << endl;
            count++;
        }
        
    }else{
        cout << "路径规划参数为空！！" << endl;
    }
   
  return 0;
}