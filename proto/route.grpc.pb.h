// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: route.proto
#ifndef GRPC_route_2eproto__INCLUDED
#define GRPC_route_2eproto__INCLUDED

#include "route.pb.h"

#include <grpc++/impl/codegen/async_stream.h>
#include <grpc++/impl/codegen/async_unary_call.h>
#include <grpc++/impl/codegen/method_handler_impl.h>
#include <grpc++/impl/codegen/proto_utils.h>
#include <grpc++/impl/codegen/rpc_method.h>
#include <grpc++/impl/codegen/service_type.h>
#include <grpc++/impl/codegen/status.h>
#include <grpc++/impl/codegen/stub_options.h>
#include <grpc++/impl/codegen/sync_stream.h>

namespace grpc {
class CompletionQueue;
class Channel;
class ServerCompletionQueue;
class ServerContext;
}  // namespace grpc

namespace haoxing {
namespace route {

class RoutingService final {
 public:
  static constexpr char const* service_full_name() {
    return "haoxing.route.RoutingService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    virtual ::grpc::Status GetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RequestParam& request, ::haoxing::route::ResponseParam* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::ResponseParam>> AsyncGetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RequestParam& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::ResponseParam>>(AsyncGetNodeXYZListRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::ResponseParam>> PrepareAsyncGetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RequestParam& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::ResponseParam>>(PrepareAsyncGetNodeXYZListRaw(context, request, cq));
    }
  private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::ResponseParam>* AsyncGetNodeXYZListRaw(::grpc::ClientContext* context, const ::haoxing::route::RequestParam& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::ResponseParam>* PrepareAsyncGetNodeXYZListRaw(::grpc::ClientContext* context, const ::haoxing::route::RequestParam& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel);
    ::grpc::Status GetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RequestParam& request, ::haoxing::route::ResponseParam* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::haoxing::route::ResponseParam>> AsyncGetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RequestParam& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::haoxing::route::ResponseParam>>(AsyncGetNodeXYZListRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::haoxing::route::ResponseParam>> PrepareAsyncGetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RequestParam& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::haoxing::route::ResponseParam>>(PrepareAsyncGetNodeXYZListRaw(context, request, cq));
    }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    ::grpc::ClientAsyncResponseReader< ::haoxing::route::ResponseParam>* AsyncGetNodeXYZListRaw(::grpc::ClientContext* context, const ::haoxing::route::RequestParam& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::haoxing::route::ResponseParam>* PrepareAsyncGetNodeXYZListRaw(::grpc::ClientContext* context, const ::haoxing::route::RequestParam& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_GetNodeXYZList_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    virtual ::grpc::Status GetNodeXYZList(::grpc::ServerContext* context, const ::haoxing::route::RequestParam* request, ::haoxing::route::ResponseParam* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_GetNodeXYZList : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_GetNodeXYZList() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_GetNodeXYZList() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetNodeXYZList(::grpc::ServerContext* context, const ::haoxing::route::RequestParam* request, ::haoxing::route::ResponseParam* response) final override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetNodeXYZList(::grpc::ServerContext* context, ::haoxing::route::RequestParam* request, ::grpc::ServerAsyncResponseWriter< ::haoxing::route::ResponseParam>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_GetNodeXYZList<Service > AsyncService;
  template <class BaseClass>
  class WithGenericMethod_GetNodeXYZList : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_GetNodeXYZList() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_GetNodeXYZList() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetNodeXYZList(::grpc::ServerContext* context, const ::haoxing::route::RequestParam* request, ::haoxing::route::ResponseParam* response) final override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_GetNodeXYZList : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithStreamedUnaryMethod_GetNodeXYZList() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler< ::haoxing::route::RequestParam, ::haoxing::route::ResponseParam>(std::bind(&WithStreamedUnaryMethod_GetNodeXYZList<BaseClass>::StreamedGetNodeXYZList, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithStreamedUnaryMethod_GetNodeXYZList() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status GetNodeXYZList(::grpc::ServerContext* context, const ::haoxing::route::RequestParam* request, ::haoxing::route::ResponseParam* response) final override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedGetNodeXYZList(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::haoxing::route::RequestParam,::haoxing::route::ResponseParam>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_GetNodeXYZList<Service > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_GetNodeXYZList<Service > StreamedService;
};

}  // namespace route
}  // namespace haoxing


#endif  // GRPC_route_2eproto__INCLUDED