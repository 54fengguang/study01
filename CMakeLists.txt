cmake_minimum_required(VERSION 3.10.0)

project(study01)


set(THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
set(Protobuf_DIR  "${THIRDPARTY_DIR}/protobuf")

add_subdirectory(third_party)

find_library(
    Protobuf_LIBRARIES protobuf 
    PATHS  "/usr/local/include"
    PATH_SUFFIXES   google
)

find_program(
    PROTOBUF_PROTOC_EXECUTABLE  protoc 
    PATHS  "/usr/local/bin"
    PATH_SUFFIXES   bin
)

find_path(
    Protobuf_INCLUDE_DIRS  google
    PATHS  "/usr/local/include"
    PATH_SUFFIXES   include
)

if("${Protobuf_LIBRARIES}" STREQUAL "Protobuf_LIBRARIES-NOTFOUND")
  message(WARNING "Protobuf package not found")
else()
  message(STATUS "Protobuf include path : ${Protobuf_INCLUDE_DIRS}")
  message(STATUS "Protobuf libraries : ${Protobuf_LIBRARIES}")
endif()


#get_filename_component(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR} DIRECTORY)

set(PROTO_META_BASE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/proto)

message(STATUS   "PROTO_META_BASE_DIR  value is :   ${PROTO_META_BASE_DIR}")
#设置输出路径
#set(MESSAGE_DIR ${CMAKE_BINARY_DIR}/proto)


#f(EXISTS "${MESSAGE_DIR}" AND IS_DIRECTORY "${MESSAGE_DIR}")
#       SET(DST_DIR ${MESSAGE_DIR})
#else()
#        file(MAKE_DIRECTORY ${MESSAGE_DIR})
#        SET(DST_DIR ${MESSAGE_DIR})
#endif()


add_subdirectory(proto)

#设置protoc的搜索路径
LIST(APPEND PROTO_FLAGS  -I${PROTO_INCLUDE_DIRS})

#获取需要编译的proto文件
set(MESSAGE_SRC "")
set(MESSAGE_HDRS "")

foreach(msg ${PROTO_FILES})

        message(STATUS  "protobuf  file is  :    ${msg}")
        get_filename_component(FIL_WE ${msg} NAME_WE)

        list(APPEND MESSAGE_SRC "${PROJECT_BINARY_DIR}/proto/${FIL_WE}.pb.cc")
        list(APPEND MESSAGE_HDRS "${PROJECT_BINARY_DIR}/proto/${FIL_WE}.pb.h")
        
        # 生成源码
        execute_process(
            COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}  ${PROTO_FLAGS} --cpp_out=${PROTO_META_BASE_DIR}    ${msg}
            )
endforeach()


if (WITH_GRPC)
    find_package(gRPC CONFIG)
    # First attempt to set up gRPC via cmake; but if cmake config files aren't
    # available, fallback to pkg-config.
    if (gRPC_FOUND)
        set(GRPC_CPP_PLUGIN $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
        list(APPEND LIGHTSTEP_LINK_LIBRARIES gRPC::grpc++)
        include_directories(SYSTEM
                $<TARGET_PROPERTY:gRPC::grpc++,INTERFACE_INCLUDE_DIRECTORIES>)
    else()
        message("Falling back to finding gRPC with pkg-config")
        find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
        if (NOT GRPC_CPP_PLUGIN)
            message(FATAL_ERROR "grpc_cpp_plugin not found!")
        endif()
        find_package(PkgConfig REQUIRED)
        pkg_search_module(GRPC REQUIRED grpc)
        pkg_search_module(GRPCPP REQUIRED grpc++)
        list(APPEND LIGHTSTEP_LINK_LIBRARIES ${GRPCPP_LDFLAGS} ${GRPC_LDFLAGS})
        include_directories(SYSTEM ${GRPC_INCLUDE_DIRS} ${GRPCPP_INCLUDE_DIRS})
    endif()
endif()

message(STATUS "GRPCPP_INCLUDE_DIRS directory is :  ${GRPCPP_INCLUDE_DIRS}")
include_directories(
    include
    ${PROTO_META_BASE_DIR}
    ${GRPCPP_INCLUDE_DIRS}
)

aux_source_directory(./src  SRC_LIST)

add_executable(${PROJECT_NAME}  ${SRC_LIST})

target_link_libraries( ${PROJECT_NAME} 
    ${Protobuf_LIBRARIES}
)