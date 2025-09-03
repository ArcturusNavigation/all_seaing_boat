cmake_minimum_required(VERSION 3.8)
project(all_seaing_lab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/buoy_course.py
    ${PROJECT_NAME}/pid_controller.py
    ${PROJECT_NAME}/sim_engine.py
    ${PROJECT_NAME}/teleop_controller.py
    ${PROJECT_NAME}/waypoint_follower.py
    ${PROJECT_NAME}/follow_path.py
    ${PROJECT_NAME}/follow_path_local.py
    ${PROJECT_NAME}/follow_path_global.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch config)