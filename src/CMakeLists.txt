cmake_minimum_required(VERSION 3.16)

project(TrafficRobust-DAC)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

include(FetchContent) # If not included already

FetchContent_Declare(sciplot_content
  GIT_REPOSITORY https://github.com/sciplot/sciplot.git
  GIT_TAG master)

FetchContent_GetProperties(sciplot_content)

if(NOT sciplot_content_POPULATED)
    FetchContent_Populate(sciplot_content)
endif()

include_directories(${sciplot_content_SOURCE_DIR})
include_directories(${PROJECT_ROOT}/include)

# Gurobi
include_directories($ENV{GUROBI_HOME}/include)
link_directories($ENV{GUROBI_HOME}/lib)

add_executable(TrafficRobust-DAC
    main.cpp
    DACData.cpp
    DACCSVGenerator.cpp
    Master.cpp
    Slave.cpp
    Solver.cpp
    Logger.cpp
    Viewer.cpp
    GraphPlotter.cpp
    ../test/test.cpp
)

include_directories(${PROJECT_ROOT}/test)

# SciPlot headers
target_include_directories(TrafficRobust-DAC PRIVATE
    ${sciplot_content_SOURCE_DIR}
    ${PROJECT_ROOT}/include
)

target_link_libraries(TrafficRobust-DAC
    gurobi_c++
    gurobi130
)