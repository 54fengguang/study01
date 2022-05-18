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
using ::google::protobuf::RepeatedPtrField;


using namespace haoxing::route;

namespace haoxing
{
    namespace road
    {
        bool RoadPlanClient::getNodeXYZList(GisFieldInfo &gisFieldInfo,
                            FarmTools &farmTools,Vehicle &vehicle,PlanParam &planParam,vector<NodeXYZ> &point_list)
        {
            bool flag = true;
            RoadPlanRequest request;
            request.mutable_gisfieldinfo()->MergeFrom(gisFieldInfo);
            request.mutable_farmtools()->MergeFrom(farmTools);
            request.mutable_vehicle()->MergeFrom(vehicle);
            request.mutable_planparam()->MergeFrom(planParam);
            
            RoadPlanRespond reply;
            ClientContext context;
            Status status = stub->GetNodeXYZList(&context,request,&reply);

            if (status.ok()) {
                cout<< "请求内容正确返回！！" << endl;

                RepeatedPtrField<NodeXYZ >  nodeXYZ_list = reply.nodes();
                for(RepeatedPtrField<NodeXYZ>::iterator iter = nodeXYZ_list.begin();iter != nodeXYZ_list.end();iter++){
                    NodeXYZ nodeXYZ;

                    nodeXYZ.set_x(iter->x());
                    nodeXYZ.set_y(iter->y());
                    nodeXYZ.set_z(iter->z());
                    nodeXYZ.set_yaw(iter->yaw());
                    nodeXYZ.set_v(iter->v());
                    nodeXYZ.set_s(iter->s());
                    nodeXYZ.set_curvature(iter->curvature());
                    nodeXYZ.set_back(iter->back());
                    nodeXYZ.set_s(iter->str_cur());
                    nodeXYZ.set_s(iter->reverse());
                    nodeXYZ.set_s(iter->type());
                    
                    //cout << iter->x() << "," << iter->y() << "," << iter->z() << "," << iter->v() << "," << iter->yaw()<< endl;

                    point_list.push_back(nodeXYZ);
                }
            }else{
                flag = false;
            } 
            return flag;
        }
        
    } // namespace road
    
} // namespace haoxing
