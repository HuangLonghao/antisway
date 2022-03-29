cmake_minimum_required(VERSION 2.8.12)

# Every project needs a name.  We call this the "examples" project.
project(butter_test)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g") 
include_directories(/usr/include/python2.7) # -l

add_executable(butter_test main.cpp Butter1.cpp)
target_link_libraries(butter_test python2.7 boost_system boost_filesystem boost_serialization boost_regex)
