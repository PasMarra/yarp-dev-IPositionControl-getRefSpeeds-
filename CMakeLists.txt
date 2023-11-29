cmake_minimum_required(VERSION 3.20)

project(module-test
        LANGUAGES CXX
        VERSION 0.0.1)

set(EXE_TARGET_NAME module-test)

# YARP
find_package(YARP REQUIRED COMPONENTS
                           dev
                           os
                           sig
                           idl_tools
                           eigen
)

# Eigen3
find_package(Eigen3 REQUIRED)

# Executable
add_executable(${EXE_TARGET_NAME}
                ./main.cpp
)

target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         YARP::YARP_dev
                                         YARP::YARP_init
                                         YARP::YARP_os
                                         YARP::YARP_sig
                                         YARP::YARP_eigen
                                         Eigen3::Eigen
)

# Install the executable
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
