#include <memory>
#include <string>
#include <iostream>

#include <grpc++/grpc++.h>
#include <grpc/grpc.h>
#include <grpc++/server.h>
#include <grpc++/server_builder.h>
#include <grpc++/server_context.h>

#include  "../proto/route.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using namespace std;