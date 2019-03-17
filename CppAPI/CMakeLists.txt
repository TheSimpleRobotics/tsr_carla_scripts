# Check Cmake version
cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Set the name of the executalbe/library
set(TARGET "carla_cpp_api")
# Flag which add thread lib
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CARLA_PATH $ENV{HOME}/tsr/carla/PythonAPI/dependencies)
set(BOOST_PATH $ENV{HOME}/tsr/carla/Build/boost-1.69.0-install)
set(GOOGLE_TST_PATH $ENV{HOME}/tsr/carla/Build/googletest-1.8.0-ex-libstdcxx-install)
set(LLVM_PATH $ENV{HOME}/tsr/carla/Build/llvm-6.0-ex-install)
set(RPC_PATH $ENV{HOME}/tsr/carla/Build/rpclib-v2.2.1_c1-libstdcxx-install)

# Define the name of project
project(${TARGET}_project)

# Include directories
include_directories(${CARLA_PATH}/include 
                    ${BOOST_PATH}/include
                    ${GOOGLE_TST_PATH}/include
                    ${LLVM_PATH}/include
                    ${RPC_PATH}/include 
                    )

link_directories(${CARLA_PATH}/lib
                 ${BOOST_PATH}/lib
                 ${GOOGLE_TST_PATH}/lib
                 ${LLVM_PATH}/lib
                 ${RPC_PATH}/lib
                 )

add_executable(${TARGET} main.cpp)

# Add libraries (CARLA_CLIENT_LIB is path to carla_client.a defined in .bashrc)
target_link_libraries(${TARGET} PRIVATE  carla_client.a 
librpc.a libc++.a libc++abi.a)
