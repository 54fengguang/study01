#include <iostream>
#include <memory>
#include <string>
#include <grpcpp/grpcpp.h>

#include "proto/route.grpc.pb.h"
#include "proto/route.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using namespace grpc;
using namespace haoxing::route;
using namespace std;

// Logic and data behind the server's behavior.
class GreeterServiceImpl  : public haoxing::route::RoutingService{

        Status   GetNodeXYZList(ServerContext* context, const RequestParam* request, ResponseParam* response) override {

                cout << "cehshi " << endl;
                return Status::OK;
         }

};
  

void RunServer() {
  
  string server_address("0.0.0.0:5001");
  
  GreeterServiceImpl service;

  ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  // 监听给定的地址
  builder.AddListeningPort(server_address, InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(&service);
  // Finally assemble the server.
  unique_ptr<Server> server(builder.BuildAndStart());
  cout << "Server listening on " << server_address <<  endl;

  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
};

int main(int argc, char** argv) {
        RunServer();

        return 0;
}