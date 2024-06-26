cmake_minimum_required(VERSION 3.10)
project(qm_controllers)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        qm_common
        qm_interface
        qm_estimation
        qm_wbc
        qm_msgs
        controller_interface
        ocs2_legged_robot_ros
        angles
        dynamic_reconfigure
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

generate_dynamic_reconfigure_options(
      cfg/weight.cfg
)    

catkin_package(
        INCLUDE_DIRS
        include
            ${EIGEN3_INCLUDE_DIRS}
        LIBRARIES
            ${PROJECT_NAME}
        CATKIN_DEPENDS
            roscpp
            qm_common
            qm_interface
            qm_estimation
            qm_wbc
            qm_msgs
            controller_interface
            ocs2_legged_robot_ros
            angles
            dynamic_reconfigure
        DEPENDS
            pinocchio
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

# Declare a cpp library
add_library(${PROJECT_NAME}
        src/QMController.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


###########
##  EXE  ##
###########
add_executable(qm_target_trajectories_publisher
     src/QmTargetTrajectoriesPublisher_node.cpp
#      src/myQmTargetTrajPub.cpp
     src/QmTargetTrajectoriesPublisher.cpp
     src/GaitJoyPublisher.cpp
     )
add_dependencies(qm_target_trajectories_publisher
     ${catkin_EXPORTED_TARGETS}
     )
target_link_libraries(qm_target_trajectories_publisher
     ${catkin_LIBRARIES}
     )
target_compile_options(qm_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

add_executable(qm_target_trajectories_publisher2
#      src/QmTargetTrajectoriesPublisher_node.cpp
     src/myQmTargetTrajPub.cpp
     src/QmTargetTrajectoriesPublisher.cpp
     src/GaitJoyPublisher.cpp
     )
add_dependencies(qm_target_trajectories_publisher2
     ${catkin_EXPORTED_TARGETS}
     )
target_link_libraries(qm_target_trajectories_publisher2
     ${catkin_LIBRARIES}
     )
target_compile_options(qm_target_trajectories_publisher2 PRIVATE ${OCS2_CXX_FLAGS})

add_executable(qm_target_trajectories_publisher3
#      src/QmTargetTrajectoriesPublisher_node.cpp
     src/my2.cpp
     src/QmTargetTrajectoriesPublisher.cpp
     src/GaitJoyPublisher.cpp
     )
add_dependencies(qm_target_trajectories_publisher3
     ${catkin_EXPORTED_TARGETS}
     )
target_link_libraries(qm_target_trajectories_publisher3
     ${catkin_LIBRARIES}
     )
target_compile_options(qm_target_trajectories_publisher3 PRIVATE ${OCS2_CXX_FLAGS})

add_executable(qm_target_trajectories_publisher4
#      src/QmTargetTrajectoriesPublisher_node.cpp
     src/my3.cpp
     src/QmTargetTrajectoriesPublisher.cpp
     src/GaitJoyPublisher.cpp
     )
add_dependencies(qm_target_trajectories_publisher4
     ${catkin_EXPORTED_TARGETS}
     )
target_link_libraries(qm_target_trajectories_publisher4
     ${catkin_LIBRARIES}
     )
target_compile_options(qm_target_trajectories_publisher4 PRIVATE ${OCS2_CXX_FLAGS})

add_executable(marker_pose_publisher src/MarkerPosePublisher.cpp)
target_link_libraries(marker_pose_publisher ${catkin_LIBRARIES})
add_dependencies(marker_pose_publisher
     ${catkin_EXPORTED_TARGETS}
     )

add_executable(gait_switch src/GaitSwitch.cpp)
target_link_libraries(gait_switch ${catkin_LIBRARIES})
add_dependencies(gait_switch
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(door_push src/DoorPush.cpp)
target_link_libraries(door_push ${catkin_LIBRARIES})
add_dependencies(door_push
        ${catkin_EXPORTED_TARGETS}
        )
add_executable(door_pull src/DoorPull.cpp)
target_link_libraries(door_pull ${catkin_LIBRARIES})
add_dependencies(door_pull
        ${catkin_EXPORTED_TARGETS}
        )




