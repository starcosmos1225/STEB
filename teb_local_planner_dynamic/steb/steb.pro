TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    ../src/graph_search.cpp \
    ../src/homotopy_class_planner.cpp \
    ../src/local_goal_planner.cpp \
    ../src/obstacles.cpp \
    ../src/optimal_planner.cpp \
    ../src/recovery_behaviors.cpp \
    ../src/teb_config.cpp \
    ../src/teb_local_planner_ros.cpp \
    ../src/test_optim_node.cpp \
    ../src/timed_elastic_band.cpp \
    ../src/visualization.cpp
INCLUDEPATH += ../include/
INCLUDEPATH += /opt/ros/kinetic/include
INCLUDEPATH += /usr/include/eigen3
HEADERS += \
    ../include/teb_local_planner_dynamic/distance_calculations.h \
    ../include/teb_local_planner_dynamic/dynamic_obstacles.h \
    ../include/teb_local_planner_dynamic/equivalence_relations.h \
    ../include/teb_local_planner_dynamic/graph_search.h \
    ../include/teb_local_planner_dynamic/homotopy_class_planner.h \
    ../include/teb_local_planner_dynamic/homotopy_class_planner.hpp \
    ../include/teb_local_planner_dynamic/h_signature.h \
    ../include/teb_local_planner_dynamic/local_goal_planner.h \
    ../include/teb_local_planner_dynamic/misc.h \
    ../include/teb_local_planner_dynamic/obstacles.h \
    ../include/teb_local_planner_dynamic/optimal_planner.h \
    ../include/teb_local_planner_dynamic/planner_interface.h \
    ../include/teb_local_planner_dynamic/pose_se2.h \
    ../include/teb_local_planner_dynamic/pose_se3.h \
    ../include/teb_local_planner_dynamic/recovery_behaviors.h \
    ../include/teb_local_planner_dynamic/robot_footprint_model.h \
    ../include/teb_local_planner_dynamic/teb_config.h \
    ../include/teb_local_planner_dynamic/teb_local_planner_ros.h \
    ../include/teb_local_planner_dynamic/timed_elastic_band.h \
    ../include/teb_local_planner_dynamic/timed_elastic_band.hpp \
    ../include/teb_local_planner_dynamic/visualization.h \
    ../include/teb_local_planner_dynamic/visualization.hpp \
    ../include/teb_local_planner_dynamic/g2o_types/base_teb_edges.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_acceleration.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_dynamic_obstacle.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_goal.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_kinematics.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_leadhuman.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_local_obstacle.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_obstacle.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_plan.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_prefer_rotdir.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_shortest_path.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_shortest_path_3D.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_time_limit.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_time_optimal.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_velocity.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_velocity_obstacle_ratio.h \
    ../include/teb_local_planner_dynamic/g2o_types/edge_via_point.h \
    ../include/teb_local_planner_dynamic/g2o_types/penalties.h \
    ../include/teb_local_planner_dynamic/g2o_types/vertex_pose.h \
    ../include/teb_local_planner_dynamic/g2o_types/vertex_pose_2d.h \
    ../include/teb_local_planner_dynamic/g2o_types/vertex_timediff.h
