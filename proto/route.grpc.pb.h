// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: route.proto
#ifndef GRPC_route_2eproto__INCLUDED
#define GRPC_route_2eproto__INCLUDED

#include "route.pb.h"

#include <functional>
#include <grpcpp/impl/codegen/async_generic_service.h>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/client_context.h>
#include <grpcpp/impl/codegen/completion_queue.h>
#include <grpcpp/impl/codegen/message_allocator.h>
#include <grpcpp/impl/codegen/method_handler.h>
#include <grpcpp/impl/codegen/proto_utils.h>
#include <grpcpp/impl/codegen/rpc_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/status.h>
#include <grpcpp/impl/codegen/stub_options.h>
#include <grpcpp/impl/codegen/sync_stream.h>

namespace haoxing {
namespace route {

class RoadPlanService final {
 public:
  static constexpr char const* service_full_name() {
    return "haoxing.route.RoadPlanService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    virtual ::grpc::Status GetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest& request, ::haoxing::route::RoadPlanRespond* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::RoadPlanRespond>> AsyncGetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::RoadPlanRespond>>(AsyncGetNodeXYZListRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::RoadPlanRespond>> PrepareAsyncGetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::RoadPlanRespond>>(PrepareAsyncGetNodeXYZListRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      virtual void GetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest* request, ::haoxing::route::RoadPlanRespond* response, std::function<void(::grpc::Status)>) = 0;
      virtual void GetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest* request, ::haoxing::route::RoadPlanRespond* response, ::grpc::ClientUnaryReactor* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::RoadPlanRespond>* AsyncGetNodeXYZListRaw(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::haoxing::route::RoadPlanRespond>* PrepareAsyncGetNodeXYZListRaw(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status GetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest& request, ::haoxing::route::RoadPlanRespond* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::haoxing::route::RoadPlanRespond>> AsyncGetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::haoxing::route::RoadPlanRespond>>(AsyncGetNodeXYZListRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::haoxing::route::RoadPlanRespond>> PrepareAsyncGetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::haoxing::route::RoadPlanRespond>>(PrepareAsyncGetNodeXYZListRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void GetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest* request, ::haoxing::route::RoadPlanRespond* response, std::function<void(::grpc::Status)>) override;
      void GetNodeXYZList(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest* request, ::haoxing::route::RoadPlanRespond* response, ::grpc::ClientUnaryReactor* reactor) override;
     private:
      friend class Stub;
      explicit async(Stub* stub): stub_(stub) { }
      Stub* stub() { return stub_; }
      Stub* stub_;
    };
    class async* async() override { return &async_stub_; }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    class async async_stub_{this};
    ::grpc::ClientAsyncResponseReader< ::haoxing::route::RoadPlanRespond>* AsyncGetNodeXYZListRaw(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::haoxing::route::RoadPlanRespond>* PrepareAsyncGetNodeXYZListRaw(::grpc::ClientContext* context, const ::haoxing::route::RoadPlanRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_GetNodeXYZList_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    virtual ::grpc::Status GetNodeXYZList(::grpc::ServerContext* context, const ::haoxing::route::RoadPlanRequest* request, ::haoxing::route::RoadPlanRespond* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_GetNodeXYZList : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_GetNodeXYZList() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_GetNodeXYZList() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetNodeXYZList(::grpc::ServerContext* /*context*/, const ::haoxing::route::RoadPlanRequest* /*request*/, ::haoxing::route::RoadPlanRespond* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetNodeXYZList(::grpc::ServerContext* context, ::haoxing::route::RoadPlanRequest* request, ::grpc::ServerAsyncResponseWriter< ::haoxing::route::RoadPlanRespond>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_GetNodeXYZList<Service > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_GetNodeXYZList : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_GetNodeXYZList() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::haoxing::route::RoadPlanRequest, ::haoxing::route::RoadPlanRespond>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::haoxing::route::RoadPlanRequest* request, ::haoxing::route::RoadPlanRespond* response) { return this->GetNodeXYZList(context, request, response); }));}
    void SetMessageAllocatorFor_GetNodeXYZList(
        ::grpc::MessageAllocator< ::haoxing::route::RoadPlanRequest, ::haoxing::route::RoadPlanRespond>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::haoxing::route::RoadPlanRequest, ::haoxing::route::RoadPlanRespond>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_GetNodeXYZList() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetNodeXYZList(::grpc::ServerContext* /*context*/, const ::haoxing::route::RoadPlanRequest* /*request*/, ::haoxing::route::RoadPlanRespond* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* GetNodeXYZList(
      ::grpc::CallbackServerContext* /*context*/, const ::haoxing::route::RoadPlanRequest* /*request*/, ::haoxing::route::RoadPlanRespond* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_GetNodeXYZList<Service > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_GetNodeXYZList : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_GetNodeXYZList() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_GetNodeXYZList() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetNodeXYZList(::grpc::ServerContext* /*context*/, const ::haoxing::route::RoadPlanRequest* /*request*/, ::haoxing::route::RoadPlanRespond* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_GetNodeXYZList : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_GetNodeXYZList() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_GetNodeXYZList() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetNodeXYZList(::grpc::ServerContext* /*context*/, const ::haoxing::route::RoadPlanRequest* /*request*/, ::haoxing::route::RoadPlanRespond* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetNodeXYZList(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_GetNodeXYZList : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_GetNodeXYZList() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->GetNodeXYZList(context, request, response); }));
    }
    ~WithRawCallbackMethod_GetNodeXYZList() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetNodeXYZList(::grpc::ServerContext* /*context*/, const ::haoxing::route::RoadPlanRequest* /*request*/, ::haoxing::route::RoadPlanRespond* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* GetNodeXYZList(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_GetNodeXYZList : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_GetNodeXYZList() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::haoxing::route::RoadPlanRequest, ::haoxing::route::RoadPlanRespond>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::haoxing::route::RoadPlanRequest, ::haoxing::route::RoadPlanRespond>* streamer) {
                       return this->StreamedGetNodeXYZList(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_GetNodeXYZList() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status GetNodeXYZList(::grpc::ServerContext* /*context*/, const ::haoxing::route::RoadPlanRequest* /*request*/, ::haoxing::route::RoadPlanRespond* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedGetNodeXYZList(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::haoxing::route::RoadPlanRequest,::haoxing::route::RoadPlanRespond>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_GetNodeXYZList<Service > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_GetNodeXYZList<Service > StreamedService;
};

}  // namespace route
}  // namespace haoxing


#endif  // GRPC_route_2eproto__INCLUDED
