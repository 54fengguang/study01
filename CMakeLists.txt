cmake_minimum_required(VERSION 3.10.0)

project(study01)

find_package(Protobuf  REQUIRED)

if("${Protobuf_LIBRARIES}" STREQUAL "Protobuf_LIBRARIES-NOTFOUND")
  message(WARNING "Protobuf package not found")
else()
  message(STATUS "Protobuf include path : ${Protobuf_INCLUDE_DIRS}")
  message(STATUS "Protobuf libraries : ${Protobuf_LIBRARIES}")
endif()



find_package(gRPC CONFIG  REQUIRED)

message(STATUS "Using gRPC ${gRPC_VERSION}")

message(STATUS "gRPC include path : ${gRPC_INCLUDE_DIRS}")
message(STATUS "gRPC libraries : ${gRPC_LIBRARIES}")



#get_filename_component(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR} DIRECTORY)

set(PROTO_META_BASE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/proto)

message(STATUS   "PROTO_META_BASE_DIR  value is :   ${PROTO_META_BASE_DIR}")


add_subdirectory(proto)

#设置protoc的搜索路径
LIST(APPEND PROTO_FLAGS  -I${PROTO_META_BASE_DIR})

foreach(FILE  ${PROTO_FILES})

        message(STATUS  "protobuf  file is  :    ${FILE}")
        
        # 生成protobuf 
        execute_process(
            COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}  ${PROTO_FLAGS}  --cpp_out=${PROJECT_SOURCE_DIR}/proto    ${FILE}
            )
        
         # 生成grpc
         execute_process(
            COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}  ${PROTO_FLAGS}  --grpc_out=${PROJECT_SOURCE_DIR}/proto  --plugin=protoc-gen-grpc=/usr/local/bin/grpc_cpp_plugin   ${FILE}
            )
endforeach()


include_directories(
    ./include
    ${Protobuf_INCLUDE_DIRS}
    ./proto
)

aux_source_directory(./src  SRC_LIST)
aux_source_directory(./proto  PROTO_LIST)


message(STATUS  "SRC_LIST  directory is :  ${SRC_LIST}")
message(STATUS  "PROTO_LIST  directory is :  ${PROTO_LIST}")

link_directories(${PROJECT_SOURCE_DIR}/libs/)

# add_executable(Path_poins  ./src/road_plan_run_service.cpp )

# target_link_libraries( Path_poins
#   librouting.so
# )


add_executable(Server  ./src/server.cpp  ./src/route_server.cpp ./src/road_plan_run_service.cpp  ${PROTO_LIST})

target_link_libraries( Server
    ${Protobuf_LIBRARIES}
     gRPC::grpc++
     librouting.so
)


add_executable(Client  ./src/client.cpp ./src/route_client.cpp  ${PROTO_LIST})

target_link_libraries( Client
    ${Protobuf_LIBRARIES}
    gRPC::grpc++
)
