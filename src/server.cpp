#include <iostream>
#include <memory>
#include <string>
 
#include <grpc++/grpc++.h>
#include <grpc/grpc.h>
#include <grpc++/server.h>
#include <grpc++/server_builder.h>
#include <grpc++/server_context.h>
 
#include "example.grpc.pb.h"
#include "routeServer.h"
 
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using namespace std;
using  haoxing::route1::RoutingServiceImpl;
 
class SearchRequestImpl final : public SearchService::Service {
  Status Search(ServerContext* context, const SearchRequest* request,
                  SearchResponse* reply) override {
    string prefix("Hello ");
    cout <<  "请求内容： " <<  request->request() << endl;
    reply->set_response(prefix + request->request());
    return Status::OK;
  }
};
 
void RunServer() {
  std::string server_address("0.0.0.0:5001");
  SearchRequestImpl  searchService;
  RoutingServiceImpl  routeingService;

 
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&searchService);
  builder.RegisterService(&routeingService);

  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;
 
  server->Wait();
}
 
int main(int argc, char** argv) {
  RunServer();
 
  return 0;
}