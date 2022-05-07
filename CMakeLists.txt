cmake_minimum_required(VERSION 3.10.0)

project(study01)


set(THIRDPARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
set(Protobuf_DIR  "${THIRDPARTY_DIR}/protobuf")

add_subdirectory(third_party)

find_library(
    Protobuf_LIBRARIES protobuf 
    PATHS  ${Protobuf_DIR}
    PATH_SUFFIXES   lib
    NO_DEFAULT_PATH
)

find_program(
    PROTOBUF_PROTOC_EXECUTABLE  protoc 
    PATHS  ${Protobuf_DIR}
    PATH_SUFFIXES   bin
    NO_DEFAULT_PATH
)

find_path(
    Protobuf_INCLUDE_DIRS  google
    PATHS  ${Protobuf_DIR}
    PATH_SUFFIXES   include()
    NO_DEFAULT_PATH
)

if("${Protobuf_LIBRARIES}" STREQUAL "Protobuf_LIBRARIES-NOTFOUND")
  message(WARNING "Protobuf package not found")
else()
  message(STATUS "Protobuf include path : ${Protobuf_INCLUDE_DIRS}")
  message(STATUS "Protobuf libraries : ${Protobuf_LIBRARIES}")
endif()


get_filename_component(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR}
                       DIRECTORY)

message(WARNING ${PROTO_META_BASE_DIR})
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
        get_filename_component(FIL_WE ${msg} NAME_WE)

        list(APPEND MESSAGE_SRC "${PROJECT_BINARY_DIR}/proto/${FIL_WE}.pb.cc")
        list(APPEND MESSAGE_HDRS "${PROJECT_BINARY_DIR}/proto/${FIL_WE}.pb.h")
        
        # 生成源码
        execute_process(
            COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS} --cpp_out=${PROTO_META_BASE_DIR}    ${msg}
            )
endforeach()

include_directories(include
    ${PROTO_META_BASE_DIR}
)

aux_source_directory(./src  SRC_LIST)

add_executable(${PROJECT_NAME}  ${SRC_LIST})

target_link_libraries( ${PROJECT_NAME} 
    ${Protobuf_LIBRARIES}
)