cmake_minimum_required(VERSION 3.10.0)
project(project_vehicle VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-fexec-charset=UTF-8 -finput-charset=UTF-8)

# 1. 源文件
file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES} ${MAIN_SRC})

# 2. 头文件 (干干净净，只有三个路径)
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/eigen3.4.0
    ${CMAKE_CURRENT_SOURCE_DIR}/osqp_bin/include
)

# 3. 链接动态库
add_library(osqp_dll SHARED IMPORTED)
set_target_properties(osqp_dll PROPERTIES
    IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/osqp_bin/lib/libosqp.dll.a"
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/osqp_bin/lib/libosqp.dll"
)
target_link_libraries(${PROJECT_NAME} PRIVATE osqp_dll)