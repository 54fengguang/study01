#include <memory>
#include <string>
#include <iostream>

#include <grpc++/grpc++.h>
#include <grpc/grpc.h>
#include <grpc++/server.h>
#include <grpc++/server_builder.h>
#include <grpc++/server_context.h>

#include  "route.grpc.pb.h"
#include "routeServer.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using namespace std;

namespace haoxing{
 namespace route1
  {
        RoutingServiceImpl::RoutingServiceImpl(){
              cout <<  "RoutingServiceImpl  构造函数调用！！" << endl;
        };


      RoutingServiceImpl::~RoutingServiceImpl(){
          cout <<  "RoutingServiceImpl  析构函数调用！！" << endl;

      };

        Status  RoutingServiceImpl::GetNodeXYZList(ServerContext* context, 
                                         const  RequestParam* request,  ResponseParam* response){   
            vector<NodeXYZ>  node_list;
            for (int  i = 0; i < 200; i++)
            {
                NodeXYZ node_xyz;
                node_xyz.set_x(102.3) ;// 横坐标
                node_xyz.set_y(112.5);  // 纵坐标
                node_xyz.set_z(113.5);// 高程
                node_xyz.set_yaw(0.345);  // 航向角
                node_xyz.set_curvature(0.009); //curvature 曲率
                node_xyz.set_segment(18);  // 段号
                node_xyz.set_reverse(false);  // 倒车
                node_xyz.set_node_type(1); // 点类型：1 田块，2机耕道
                node_xyz.set_impl_plough_ori(14); //犁翻转方向
                node_xyz.set_impl_state(2); //农具工作类型：1.不作业，2.作业

              cout << "输出第:  " << i << " 个坐标点:"   << node_xyz.x()  <<  node_xyz.y() << endl;
              node_list.push_back(node_xyz);
            }
            response->mutable_nodes()->CopyFrom({node_list.begin(),node_list.end()});
            response ->set_mileage(100.0);
            return Status::OK;
        };
  } // namespace route  
} // namespace haoxin

