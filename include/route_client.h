#include <vector>
 
#include <grpc++/grpc++.h>
#include "route.grpc.pb.h"

using grpc::Channel;

using namespace std;
using namespace haoxing::route;

namespace haoxing
{
    namespace road
    {
        class RoadPlanClient
        {
            public:
                explicit RoadPlanClient(shared_ptr<Channel> channel) : stub(RoadPlanService::NewStub(channel)){};

              
                /**
                 * @brief 通过rpc方式获取田内作业路径规划点 NodeXYZ 列表信息
                 * 
                 * @param gisFieldInfo  田块参数信息
                 * @param obstacleReq_list 障碍物参数信息
                 * @param farmTools  农具参数信息
                 * @param vehicle   拖拉机参数信息
                 * @param planParam  田内作业路径规划参数信息
                 * @return vector<NodeXYZ> 
                 */
                bool  getNodeXYZList(GisFieldInfo &gisFieldInfo,FarmTools &farmTools,
                        Vehicle &vehicle,PlanParam &planParam,vector<NodeXYZ>  &point_list);
            
            private:
                std::unique_ptr<RoadPlanService::Stub> stub;

        };
        
    } // namespace road
    
} // namespace haoxing
