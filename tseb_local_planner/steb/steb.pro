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
    ../include/tseb_local_planner/distance_calculations.h \
    ../include/tseb_local_planner/dynamic_obstacles.h \
    ../include/tseb_local_planner/equivalence_relations.h \
    ../include/tseb_local_planner/graph_search.h \
    ../include/tseb_local_planner/homotopy_class_planner.h \
    ../include/tseb_local_planner/homotopy_class_planner.hpp \
    ../include/tseb_local_planner/h_signature.h \
    ../include/tseb_local_planner/local_goal_planner.h \
    ../include/tseb_local_planner/misc.h \
    ../include/tseb_local_planner/obstacles.h \
    ../include/tseb_local_planner/optimal_planner.h \
    ../include/tseb_local_planner/planner_interface.h \
    ../include/tseb_local_planner/pose_se2.h \
    ../include/tseb_local_planner/pose_se3.h \
    ../include/tseb_local_planner/recovery_behaviors.h \
    ../include/tseb_local_planner/robot_footprint_model.h \
    ../include/tseb_local_planner/teb_config.h \
    ../include/tseb_local_planner/teb_local_planner_ros.h \
    ../include/tseb_local_planner/timed_elastic_band.h \
    ../include/tseb_local_planner/timed_elastic_band.hpp \
    ../include/tseb_local_planner/visualization.h \
    ../include/tseb_local_planner/visualization.hpp \
    ../include/tseb_local_planner/g2o_types/base_teb_edges.h \
    ../include/tseb_local_planner/g2o_types/edge_acceleration.h \
    ../include/tseb_local_planner/g2o_types/edge_dynamic_obstacle.h \
    ../include/tseb_local_planner/g2o_types/edge_goal.h \
    ../include/tseb_local_planner/g2o_types/edge_kinematics.h \
    ../include/tseb_local_planner/g2o_types/edge_leadhuman.h \
    ../include/tseb_local_planner/g2o_types/edge_local_obstacle.h \
    ../include/tseb_local_planner/g2o_types/edge_obstacle.h \
    ../include/tseb_local_planner/g2o_types/edge_plan.h \
    ../include/tseb_local_planner/g2o_types/edge_prefer_rotdir.h \
    ../include/tseb_local_planner/g2o_types/edge_shortest_path.h \
    ../include/tseb_local_planner/g2o_types/edge_shortest_path_3D.h \
    ../include/tseb_local_planner/g2o_types/edge_time_limit.h \
    ../include/tseb_local_planner/g2o_types/edge_time_optimal.h \
    ../include/tseb_local_planner/g2o_types/edge_velocity.h \
    ../include/tseb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h \
    ../include/tseb_local_planner/g2o_types/edge_via_point.h \
    ../include/tseb_local_planner/g2o_types/penalties.h \
    ../include/tseb_local_planner/g2o_types/vertex_pose.h \
    ../include/tseb_local_planner/g2o_types/vertex_pose_2d.h \
    ../include/tseb_local_planner/g2o_types/vertex_timediff.h
