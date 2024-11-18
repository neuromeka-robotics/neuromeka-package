cmake_minimum_required(VERSION 3.10)
project(IndyDCP3)

# C++ standard
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED True)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Build option
option(BUILD_PROTO "Build proto task" OFF)

if(WIN32)
    message(STATUS "Configuring for Windows")
    
    # Manual link cmake
    set(absl_DIR "E:/Example/install/lib/cmake/absl")
    set(utf8_range_DIR "E:/Example/install/lib/cmake/utf8_range")
    set(protobuf_DIR "E:/Example/install/cmake")
    set(gRPC_DIR "E:/Example/install/lib/cmake/grpc")

    # Find Protobuf and gRPC packages
    find_package(protobuf CONFIG REQUIRED)
    message(STATUS "Using protobuf ${Protobuf_VERSION}")

    find_package(gRPC CONFIG REQUIRED)
    message(STATUS "Using gRPC ${gRPC_VERSION}")

else()
    message(STATUS "Configuring for Linux")

    # Find Protobuf and gRPC packages
    find_package(Protobuf CONFIG REQUIRED)
    message(STATUS "Using protobuf ${Protobuf_VERSION}")

    find_package(gRPC CONFIG REQUIRED)
    message(STATUS "Using gRPC ${gRPC_VERSION}")

endif()

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_REFLECTION gRPC::grpc++_reflection)
set(_GRPC_GRPCPP gRPC::grpc++)

# Include directories for source files
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${gRPC_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/neuromeka_cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/neuromeka_cpp/proto)

if(BUILD_PROTO)
    add_executable(build_proto src/build_proto.cpp)
    target_link_libraries(build_proto ${Protobuf_LIBRARIES} ${gRPC_LIBRARIES})

else()
    set(INDY_SOURCES
        neuromeka_cpp/indydcp3.cpp
        neuromeka_cpp/proto/common_msgs.pb.cc
        neuromeka_cpp/proto/config_msgs.pb.cc
        neuromeka_cpp/proto/control_msgs.pb.cc
        neuromeka_cpp/proto/device_msgs.pb.cc
        neuromeka_cpp/proto/ethercat_msgs.pb.cc
        neuromeka_cpp/proto/moby_msgs.pb.cc
        neuromeka_cpp/proto/rtde_msgs.pb.cc
        neuromeka_cpp/proto/hri_msgs.pb.cc
        neuromeka_cpp/proto/shared_msgs.pb.cc

        neuromeka_cpp/proto/common_msgs.grpc.pb.cc
        neuromeka_cpp/proto/config_msgs.grpc.pb.cc
        neuromeka_cpp/proto/control_msgs.grpc.pb.cc
        neuromeka_cpp/proto/device_msgs.grpc.pb.cc
        neuromeka_cpp/proto/ethercat_msgs.grpc.pb.cc
        neuromeka_cpp/proto/moby_msgs.grpc.pb.cc
        neuromeka_cpp/proto/rtde_msgs.grpc.pb.cc
        neuromeka_cpp/proto/hri_msgs.grpc.pb.cc
        neuromeka_cpp/proto/shared_msgs.grpc.pb.cc

        neuromeka_cpp/proto/config.pb.cc
        neuromeka_cpp/proto/control.pb.cc
        neuromeka_cpp/proto/device.pb.cc
        neuromeka_cpp/proto/ethercat.pb.cc
        neuromeka_cpp/proto/eyetask.pb.cc
        neuromeka_cpp/proto/indyeye.pb.cc
        neuromeka_cpp/proto/moby.pb.cc
        neuromeka_cpp/proto/rtde.pb.cc
        neuromeka_cpp/proto/cri.pb.cc
        neuromeka_cpp/proto/hri.pb.cc

        neuromeka_cpp/proto/config.grpc.pb.cc
        neuromeka_cpp/proto/control.grpc.pb.cc
        neuromeka_cpp/proto/device.grpc.pb.cc
        neuromeka_cpp/proto/ethercat.grpc.pb.cc
        neuromeka_cpp/proto/eyetask.grpc.pb.cc
        neuromeka_cpp/proto/indyeye.grpc.pb.cc
        neuromeka_cpp/proto/moby.grpc.pb.cc
        neuromeka_cpp/proto/rtde.grpc.pb.cc
        neuromeka_cpp/proto/cri.grpc.pb.cc
        neuromeka_cpp/proto/hri.grpc.pb.cc
    )

    add_executable(example_indydcp3 src/example_indydcp3.cpp ${INDY_SOURCES})
    target_link_libraries(example_indydcp3 
                        ${Protobuf_LIBRARIES} 
                        ${gRPC_LIBRARIES}
                        ${_REFLECTION}
                        ${_GRPC_GRPCPP}
                        ${_PROTOBUF_LIBPROTOBUF}
                        )
endif()
