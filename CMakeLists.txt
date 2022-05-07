cmake_minimum_required(VERSION 3.10.0)

project(study01)


aux_source_directory(./src  SRC_LIST)


add_executable(${PROJECT_NAME}  ${SRC_LIST})