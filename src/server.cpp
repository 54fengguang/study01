#include <iostream>
#include <memory>
#include <string>
 
#include <grpc++/grpc++.h>
#include <grpc/grpc.h>
#include <grpc++/server.h>
#include <grpc++/server_builder.h>
#include <grpc++/server_context.h>
 
#include "route_server.h"
 
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using  haoxing::road::RoadPlanServiceImpl;

using namespace std;
 
void RunServer() {
  std::string server_address("0.0.0.0:50051");
  RoadPlanServiceImpl  roadPlanService;

 
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&roadPlanService);

  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
 
  server->Wait();
}
 
int main(int argc, char** argv) {
  RunServer();
 
  return 0;
}