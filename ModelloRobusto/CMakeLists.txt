cmake_minimum_required(VERSION 3.16)

project(TrafficRobust-DAC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Gurobi
include_directories($ENV{GUROBI_HOME}/include)
link_directories($ENV{GUROBI_HOME}/lib)

add_executable(TrafficRobust-DAC
    main.cpp
    Master.cpp
    Slave.cpp
    generate_dac_data.cpp
    DACCSVGenerator.cpp
)

target_link_libraries(TrafficRobust-DAC
    gurobi_c++
    gurobi130
)