# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(control_ui 0.1)

include(RockRuby)
rock_standard_layout()

install(PROGRAMS bin/rock-control_bot
    DESTINATION bin)

install(PROGRAMS bin/bldc_control_joints_gui
    DESTINATION bin)
