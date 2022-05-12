#include <grpc/grpc.h>
#include <grpc++/grpc++.h>
#include <grpc++/server_context.h>
#include  "route.grpc.pb.h"

using  grpc::Status;
using  grpc::ServerContext;
using namespace std;
using namespace  haoxing::route;

namespace haoxing{
    namespace route1
    {
        class RoutingServiceImpl final  :  public  RoutingService::Service {

            public:
                    RoutingServiceImpl();

                    ~RoutingServiceImpl();

                    Status GetNodeXYZList(ServerContext* context, 
                                                                        const  RequestParam* request, 
                                                                        ResponseParam* response);
        };
    } // namespace route
} // namespace haoxing

