#include <grpc/grpc.h>
#include <grpc++/grpc++.h>
#include <grpc++/server_context.h>
#include  "route.grpc.pb.h"

using  grpc::Status;
using  grpc::ServerContext;
using namespace std;
using namespace  haoxing::route;

namespace haoxing{
    namespace road
    {
        class RoadPlanServiceImpl final  :  public  RoadPlanService::Service 
        {
            private:
                bool check(const RoadPlanRequest* request);


            public:
                RoadPlanServiceImpl();

                ~RoadPlanServiceImpl();

                Status GetNodeXYZList(ServerContext* context,
                            const RoadPlanRequest* request,RoadPlanRespond* response);
        };
    } // namespace route
} // namespace haoxing

