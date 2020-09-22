/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner_dynamic/optimal_planner.h>

// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner_dynamic/g2o_types/vertex_pose.h>
#include <teb_local_planner_dynamic/g2o_types/vertex_pose_2d.h>
#include <teb_local_planner_dynamic/g2o_types/vertex_timediff.h>
#include <teb_local_planner_dynamic/g2o_types/edge_velocity.h>
#include <teb_local_planner_dynamic/g2o_types/edge_velocity_obstacle_ratio.h>
#include <teb_local_planner_dynamic/g2o_types/edge_acceleration.h>
#include <teb_local_planner_dynamic/g2o_types/edge_kinematics.h>
#include <teb_local_planner_dynamic/g2o_types/edge_time_optimal.h>
#include <teb_local_planner_dynamic/g2o_types/edge_shortest_path.h>
#include <teb_local_planner_dynamic/g2o_types/edge_obstacle.h>
#include <teb_local_planner_dynamic/g2o_types/edge_dynamic_obstacle.h>
#include <teb_local_planner_dynamic/g2o_types/edge_via_point.h>
#include <teb_local_planner_dynamic/g2o_types/edge_prefer_rotdir.h>
#include <teb_local_planner_dynamic/g2o_types/edge_shortest_path_3D.h>
#include <teb_local_planner_dynamic/g2o_types/edge_goal.h>
#include <teb_local_planner_dynamic/g2o_types/edge_time_limit.h>
#include <iostream>
#include <memory>
#include <limits>
#include <sys/time.h>

namespace teb_local_planner
{

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                         robot_model_(new PointRobotFootprint()), initialized_(false), optimized_(false)
{    
}
  
TebOptimalPlanner::TebOptimalPlanner(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  initialize(cfg, obstacles, robot_model, visual, via_points);
}

TebOptimalPlanner::~TebOptimalPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_) 
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void TebOptimalPlanner::initialize(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();
  
  cfg_ = &cfg;
  obstacles_ = obstacles;
  robot_model_ = robot_model;
  via_points_ = via_points;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;
  setVisualization(visual);
  
  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;
  initialized_ = true;
   time_diff_goal = new VertexPose;
}


void TebOptimalPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize()
{
  if (!visualization_)
    return;
 
  visualization_->publishLocalPlanAndPoses(teb_);
  PoseSE2 pose = PoseSE2(teb_.Pose(0).position(),teb_.Pose(0).theta());
  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(pose, *robot_model_);
  else if (teb_.PoseGoal())
      visualization_->publishRobotFootprintModel(teb_.PoseGoal()->pose(),*robot_model_);
  
  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);
 
}


/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);
  factory->registerType("VERTEX_POSE_2D", new g2o::HyperGraphElementCreator<VertexPose2D>);

  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
  //factory->registerType("EDGE_TIME_OPTIMAL_GOAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimalGoal>);
  factory->registerType("EDGE_TIME_LIMIT",new g2o::HyperGraphElementCreator<EdgeTimeLimit>);
  factory->registerType("EDGE_SHORTEST_PATH", new g2o::HyperGraphElementCreator<EdgeShortestPath>);
  factory->registerType("EDGE_SHORTEST_PATH_GOAL", new g2o::HyperGraphElementCreator<EdgeShortestPathGoal>);
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
  factory->registerType("EDGE_VELOCITY_GOAL", new g2o::HyperGraphElementCreator<EdgeVelocityGoal>);
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>);
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
  factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE_GOAL", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDriveGoal>);
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
  factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>);
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
  factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>);
  factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);
  factory->registerType("EDGE_SHORTEST_PATH_3D",new g2o::HyperGraphElementCreator<EdgeShortestPath3D>);
  factory->registerType("EDGE_SHORTEST_PATH_3D_GOAL",new g2o::HyperGraphElementCreator<EdgeShortestPath3DGoal>);
  factory->registerType("EDGE_GOAL",new g2o::HyperGraphElementCreator<EdgeGoal>);
  factory->registerType("EDGE_TIME_GOAL",new g2o::HyperGraphElementCreator<EdgeTimeOptimalGoal>);

  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypes, flag);  

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  TEBLinearSolver* linearSolver = new TEBLinearSolver(); // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);
  TEBBlockSolver* blockSolver = new TEBBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

  optimizer->setAlgorithm(solver);
  
  optimizer->initMultiThreading(); // required for >Eigen 3.1
  
  return optimizer;
}


bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop,geometry_msgs::PoseStamped* human_pose, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  //ROS_INFO("begin optimize teb");
    timeval t_start, t_end;
    gettimeofday( &t_start, NULL);
    double delta_t;

  if (cfg_->optim.optimization_activate==false) 
    return false;
  
  bool success = false;
  optimized_ = false;
  
  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;
  //for (int i=0;i<teb_.sizePoses();++i)
  //{
    //  ROS_INFO("begin op index:%d x:%lf y:%lf t:%lf",i,teb_.Pose(i).x(),teb_.Pose(i).y(),teb_.Pose(i).t());
  //}
  //ROS_INFO("begin optimize TEB compute");
  //if (compute_cost_afterwards)
      //ROS_INFO("cost compute :true");
  //else
      //ROS_INFO("cost compute :false");
  //ROS_INFO("goal info:%lf %lf",goal_.x(),goal_.y());
  //goal_ = teb_.BackPose();
  for(int i=0; i<iterations_outerloop; ++i)
  {
      // for (auto &a:teb_.poses())
      // {
      //     ROS_INFO("before auto size teb point info:x:%lf y:%lf theta:%lf t:%lf",a->x(),a->y(),a->theta(),a->t());
      // }
      //ROS_INFO("before auto size teb point info:x:%lf y:%lf theta:%lf t:%lf",teb_.PoseGoal()->x(),teb_.PoseGoal()->y(),teb_.PoseGoal()->theta(),teb_.BackPose().t()+teb_.timeDiffVertex()->dt());
      //ROS_INFO("begin auto size");
    if (cfg_->trajectory.teb_autosize)
    {
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples,
                      cfg_->trajectory.max_samples, fast_mode,cfg_->robot.max_vel_x,cfg_->robot.max_vel_theta);

    }
   //ROS_INFO("end auto size");
    if (i==iterations_outerloop-1)
        success = buildGraph(weight_multiplier,true);
    else
        success = buildGraph(weight_multiplier);

    if (!success) 
    {
        clearGraph();
        return false;
    }
  //  for (int j=0;j<teb_.sizePoses();++j)
  //  {
  //       ROS_INFO("before optimize:%d x:%lf y:%lf t:%lf theta:%lf",j,teb_.Pose(j).x(),teb_.Pose(j).y(),teb_.Pose(j).t(),teb_.Pose(j).theta());
  //   }
   // ROS_INFO("before optimize:x:%lf y:%lf theta:%lf t:%lf",teb_.PoseGoal()->x(),teb_.PoseGoal()->y(),teb_.PoseGoal()->theta(),teb_.BackPose().t()+teb_.timeDiffVertex()->dt());

    success = optimizeGraph(iterations_innerloop, false);

  //   for (int j=0;j<teb_.sizePoses();++j)
  //   {
  //      ROS_INFO("after optimize:%d x:%lf y:%lf t:%lf theta:%lf",j,teb_.Pose(j).x(),teb_.Pose(j).y(),teb_.Pose(j).t(),teb_.Pose(j).theta());
  //  }
    //ROS_INFO("after optimize:x:%lf y:%lf theta:%lf t:%lf",teb_.PoseGoal()->x(),teb_.PoseGoal()->y(),teb_.PoseGoal()->theta(),teb_.BackPose().t()+teb_.timeDiffVertex()->dt());

    if (!success) 
    {
        clearGraph();
        return false;
    }
    optimized_ = true;
    
    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
    {
        computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
        //ROS_INFO("current cost:%lf",cost_);
    }

      
    clearGraph();
    
    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }
  if (cfg_->leadHuman.local_pose_to_human_full&&human_pose)
  {
      //ROS_INFO("update pose optimize teb");
      teb_.updatePose(*human_pose);
  }
  //std::cout<<"plan point:"<<std::endl;
  //for (int i=0;i<teb_.sizePoses();++i)
  //{
    //std::cout<<teb_.Pose(i).x()<<" "<<teb_.Pose(i).y()<<" "<<teb_.Pose(i).t()<<std::endl;  
      //ROS_INFO("index %d: %lf %lf %lf",i,teb_.Pose(i).x(),teb_.Pose(i).y(),teb_.Pose(i).t());
  //}
  //ROS_INFO("index %d: %lf %lf %lf",teb_.sizePoses(),teb_.PoseGoal()->x(),teb_.PoseGoal()->y(),teb_.BackPose().t()+teb_.timeDiffVertex()->dt());

  return true;
}

void TebOptimalPlanner::setVelocityStart(const geometry_msgs::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void TebOptimalPlanner::setVelocityGoal(const geometry_msgs::Twist& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}
void TebOptimalPlanner::setGoal(const geometry_msgs::PoseStamped& goal)
{
    goal_=PoseSE3(goal.pose.position.x,goal.pose.position.y,0.0,0.0);
}
bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel,geometry_msgs::PoseStamped* human_pose)
{    

  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
      //ROS_WARN("initTrajectoryToGoal");
      //for (auto &p:initial_plan)
      //{
          //ROS_INFO("plan info:x:%lf y:%lf theta:%lf",p.pose.position.x,p.pose.position.y,tf::getYaw(p.pose.orientation));
      //}
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
      cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    //for (int i =0;i<teb_.sizePoses();i++)
      //    {
        //      ROS_INFO("after init teb index:%d x:%lf y:%lf theta:%lf t:%lf",i,teb_.Pose(i).x(),teb_.Pose(i).y(),teb_.Pose(i).theta(),teb_.Pose(i).t());
         //}
    goal_=PoseSE3(initial_plan.back().pose,0.0);
    //ROS_WARN("end initTrajectoryToGoal");
  }
  else // warm start
  {
    //ROS_INFO("DEBUG:inner plan");
    //for (auto &p:initial_plan)
    //{
      //  ROS_INFO("plan info:x:%lf y:%lf theta:%lf",p.pose.position.x,p.pose.position.y,tf::getYaw(p.pose.orientation));
   //}
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_p(initial_plan.back().pose);
    //for (auto &a:teb_.poses())
      //   {
        //     ROS_INFO("before prune teb point info:x:%lf y:%lf theta:%lf t:%lf",a->x(),a->y(),a->theta(),a->t());
         //}
    if (teb_.sizePoses()>0
        && (goal_p.position() - teb_.PoseGoal()->position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal_p.theta() - teb_.PoseGoal()->theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
    {
        teb_.updateAndPruneTEB(start_, goal_p, cfg_->trajectory.min_samples,cfg_->robot.max_vel_x,cfg_->robot.max_vel_theta); // update TEB
    }
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      //ROS_INFO("clear teb");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
        cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);

    }
    goal_=PoseSE3(initial_plan.back().pose,0.0);
    //for (auto &a:teb_.poses())
     //     {
       //      ROS_INFO("after prune teb point info:x:%lf y:%lf theta:%lf t:%lf",a->x(),a->y(),a->theta(),a->t());
        // }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)

  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations,human_pose);
}


bool TebOptimalPlanner::plan(const tf::Pose& start, const tf::Pose& goal,const geometry_msgs::Twist* start_vel, bool free_goal_vel,geometry_msgs::PoseStamped* human_pose)
{
  PoseSE2 start_(start);
  PoseSE2 goal_p(goal);
  return plan(start_, goal_p, start_vel,human_pose);
}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal,const geometry_msgs::Twist* start_vel, bool free_goal_vel,geometry_msgs::PoseStamped* human_pose)
{	
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    // init trajectory
    teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
    goal_=PoseSE3(goal.x(),goal.y(),0.0,goal.theta());
  }
  else // warm start
  {
    if (teb_.sizePoses() > 0
        && (goal.position() - teb_.PoseGoal()->position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal.theta() - teb_.PoseGoal()->theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples,cfg_->robot.max_vel_x,cfg_->robot.max_vel_theta);
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
        ROS_INFO("clear teb");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
      goal_=PoseSE3(goal.x(),goal.y(),0.0,goal.theta());
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
      
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations,human_pose);
}


bool TebOptimalPlanner::buildGraph(double weight_multiplier,bool debug)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }
  
  // add TEB vertices
  AddTEBVertices();
  
  // add Edges (local cost functions)
  if (cfg_->obstacles.include_costmap_obstacles)
  {
    if (cfg_->obstacles.legacy_obstacle_association)
      AddEdgesObstaclesLegacy(weight_multiplier);
    else
      AddEdgesObstacles(weight_multiplier);
  }
  if (cfg_->obstacles.include_dynamic_obstacles)
    AddEdgesDynamicObstacles(1.0,debug);
  //ROS_INFO("add edges xx");
  AddEdgesViaPoints();
  //use another vertex VertexPose2D and the VertexTimeDiff for the goal and make it fix, we need not use the edge_goal.
  //AddEdgesGoal();

  AddEdgesVelocity();
  //ROS_INFO("add edges acc");
  AddEdgesAcceleration();
  //ROS_INFO("add edges t");
  //AddEdgesTimeOptimal();
  //ROS_INFO("add edges kinetic");
  //AddEdgesShortestPath();
  AddEdgesShortestTimeSpace();
  // modified by Hu xingyu: add a onmi type For onim type we need not constrain the robot's kinematic.
  if (!cfg_->robot.onmi_type)
  {
      if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
          AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
      else
          AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.
  }
  //ROS_INFO("add edges prefer");
  AddEdgesPreferRotDir();

  if (cfg_->optim.weight_velocity_obstacle_ratio > 0)
    AddEdgesVelocityObstacleRatio();
   //ROS_INFO("end add edges ");
  return true;  
}

bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
    //ROS_INFO("begin optimize graph ");
  if (cfg_->robot.max_vel_x<0.01)
  {
    ROS_WARN("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;	
  }
  
  if (!teb_.isInit() || teb_.sizePoses()+1 < cfg_->trajectory.min_samples)
  {
    ROS_WARN("optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;	
  }
  
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if(!iter)
  {
	ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }

  if (clear_after) clearGraph();	
  //ROS_INFO("end optimize graph ");
  return true;
}

void TebOptimalPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    //optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
    optimizer_->vertices().clear();  // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}



void TebOptimalPlanner::AddTEBVertices()
{
  // add vertices to graph
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  obstacles_per_vertex_.resize(teb_.sizePoses()+1);
  auto iter_obstacle = obstacles_per_vertex_.begin();
  for (int i=0; i<teb_.sizePoses(); ++i)
  {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));
    iter_obstacle->clear();
    (iter_obstacle++)->reserve(obstacles_->size());
  }
  teb_.PoseGoal()->setId(id_counter++);
  optimizer_->addVertex(teb_.PoseGoal());
  teb_.timeDiffVertex()->setId(id_counter++);
  optimizer_->addVertex(teb_.timeDiffVertex());
  iter_obstacle->clear();
  (iter_obstacle++)->reserve(obstacles_->size());
}


void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr )
    return; // if weight equals zero skip adding edges!
    
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
  
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  auto create_edge = [inflated, &information, &information_inflated, this] (int index, const Obstacle* obstacle) {
    if (inflated)
    {
      EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;

      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
      EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    };
  };
    
  // iterate all teb points, skipping the last and, if the EdgeVelocityObstacleRatio edges should not be created, the first one too
  const int first_vertex = cfg_->optim.weight_velocity_obstacle_ratio == 0 ? 1 : 0;
  for (int i = first_vertex; i < teb_.sizePoses() - 1; ++i)
  {    
      double left_min_dist = std::numeric_limits<double>::max();
      double right_min_dist = std::numeric_limits<double>::max();
      ObstaclePtr left_obstacle;
      ObstaclePtr right_obstacle;
      
      const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();
      
      // iterate obstacles
      for (const ObstaclePtr& obst : *obstacles_)
      {
        // we handle dynamic obstacles differently below
        if(cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
          continue;

          // calculate distance to robot model
        PoseSE2 pose(teb_.Pose(i).position(),teb_.Pose(i).theta());
          double dist = robot_model_->calculateDistance(pose, obst.get());
          
          // force considering obstacle if really close to the current pose
        if (dist < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor)
          {
              iter_obstacle->push_back(obst);
              continue;
          }
          // cut-off distance
          if (dist > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor)
            continue;
          
          // determine side (left or right) and assign obstacle if closer than the previous one
          if (cross2d(pose_orient, obst->getCentroid()) > 0) // left
          {
              if (dist < left_min_dist)
              {
                  left_min_dist = dist;
                  left_obstacle = obst;
              }
          }
          else
          {
              if (dist < right_min_dist)
              {
                  right_min_dist = dist;
                  right_obstacle = obst;
              }
          }
      }   
      
      if (left_obstacle)
        iter_obstacle->push_back(left_obstacle);
      if (right_obstacle)
        iter_obstacle->push_back(right_obstacle);

      // continue here to ignore obstacles for the first pose, but use them later to create the EdgeVelocityObstacleRatio edges
      if (i == 0)
      {
        ++iter_obstacle;
        continue;
      }

      // create obstacle edges
      for (const ObstaclePtr obst : *iter_obstacle)
        create_edge(i, obst.get());
      ++iter_obstacle;
  }
}


void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information; 
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
    
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;
    
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (cfg_->obstacles.include_dynamic_obstacles && (*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue; 
    
    int index;
    
    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses())
      index =  teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));
     
    
    // check if obstacle is outside index-range between start and goal
    if ( (index <= 1) || (index > teb_.sizePoses()-2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
	    continue; 
        
    if (inflated)
    {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbourIdx=0; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index+neighbourIdx < teb_.sizePoses())
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information_inflated);
                dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information);
                dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
      }
      if ( index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information_inflated);
                dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information);
                dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
      }
    } 
    
  }
}


void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier,bool debug)
{
    //ROS_INFO("add edge dynamic");
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;
  ObstContainer dynamic_obstacles;
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
      if ((*obst)->isDynamic())
          dynamic_obstacles.push_back(*obst);
  }
  if (dynamic_obstacles.size()==0)
        return;
  debug=false;
  if (debug)
    std::cout<<"dynamic obstacles"<<std::endl;
  
  std::vector<std::pair<int,int>> ve;
  int min_point_index=-1;
  int min_obstacle_index=-1;
  double minDist=std::numeric_limits<double>::max();
  for (unsigned int i=0;i<dynamic_obstacles.size();++i)
  {
    //if (i>=cfg_->trajectory.dynamic_predict_no)
      //continue;
    if (debug)
    std::cout<<dynamic_obstacles[i]->getCentroid3D()[0]<<" "<<dynamic_obstacles[i]->getCentroid3D()[1]<<" "<<dynamic_obstacles[i]->getCentroid3D()[2]<<std::endl;
  }    
  for (int j=1;j<teb_.sizePoses();j++)
  {
    int min_obst=-1;
    double min_dist = std::numeric_limits<double>::max();
    for (unsigned int i=0;i<dynamic_obstacles.size()-1;++i)
    {
      //if (i>=cfg_->trajectory.dynamic_predict_no)
        //continue;
      if (computeDistPointToPoint(dynamic_obstacles[i]->getCentroid(),dynamic_obstacles[i+1]->getCentroid())<cfg_->obstacles.dynamic_ref_dist)
      {
        
        Eigen::Vector3d point(teb_.Pose(j).x(),teb_.Pose(j).y(),teb_.Pose(j).t());
        double scale =(dynamic_obstacles[i+1]->getCentroid()-dynamic_obstacles[i]->getCentroid() ).norm()/
                (dynamic_obstacles[i+1]->getTime()-dynamic_obstacles[i]->getTime())/cfg_->robot.max_vel_x;
        double dist=distance_point_to_segment_3d(point,dynamic_obstacles[i]->getCentroid3D(),dynamic_obstacles[i+1]->getCentroid3D(),true,scale);
        if (dist<min_dist&&dist>0)
        {
          min_obst=i;
          min_dist=dist;
        }
      }
    }
    if (min_obst>=0)//&&min_dist<cfg_->obstacles.min_obstacle_dist*4)
    {
      //ROS_INFO("add edge:%d",ve.size());
      ve.push_back(std::make_pair(min_obst,j));
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle();
      dynobst_edge->setVertex(0,teb_.PoseVertex(j));
      dynobst_edge->setInformation(information);
      std::vector<ObstaclePtr> pt(2);
      pt[0]=dynamic_obstacles[min_obst];
      pt[1]=dynamic_obstacles[min_obst+1];
      dynobst_edge->setParameters(*cfg_, robot_model_.get(),pt);
      optimizer_->addEdge(dynobst_edge);
    }
    
  }
  // for (unsigned int i=0;i<dynamic_obstacles.size()-1;++i)
  // {
  //   if (i>=cfg_->trajectory.dynamic_predict_no)
  //     continue;
  //   if (debug)
  //   std::cout<<dynamic_obstacles[i]->getCentroid3D()[0]<<" "<<dynamic_obstacles[i]->getCentroid3D()[1]<<" "<<dynamic_obstacles[i]->getCentroid3D()[2]<<std::endl;
  //     if (computeDistPointToPoint(dynamic_obstacles[i]->getCentroid(),dynamic_obstacles[i+1]->getCentroid())<cfg_->obstacles.dynamic_ref_dist)
  //     {
  //       //ROS_INFO("dynamic obstacles index:%d->%ld",i,i+1);
  //       double dist;
  //       auto points = teb_.findTrajectoryPose3DInDist(dynamic_obstacles[i]->getCentroid3D(),dynamic_obstacles[i+1]->getCentroid3D(),(cfg_->obstacles.min_obstacle_dist+0.93)*2,cfg_->optim.weight_dynamic_obstacle_time_factor);
  //       for (int index:points)
  //       {
  //         if (index>0&&index<teb_.sizePoses())
  //         {
  //           //ROS_INFO("add edge:%d",ve.size());
  //           ve.push_back(std::make_pair(i,index));
  //           EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle();
  //           dynobst_edge->setVertex(0,teb_.PoseVertex(index));
  //           dynobst_edge->setInformation(information);
  //           std::vector<ObstaclePtr> pt(2);
  //           pt[0]=dynamic_obstacles[i];
  //           pt[1]=dynamic_obstacles[i+1];
  //           dynobst_edge->setParameters(*cfg_, robot_model_.get(),pt);
  //           optimizer_->addEdge(dynobst_edge);
  //         }
  //       }
          
            
  //         //ROS_INFO("local point:%d, the distance is:%lf",index,dist);
          
  //     }
  // }
  // if (minDist<cfg_->obstacles.min_obstacle_dist*4&&min_obstacle_index!=-1&&min_point_index!=-1)
  // {
  //   ve.push_back(std::make_pair(min_obstacle_index,min_point_index));
  //             //ROS_INFO("add into teb");
  //   EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle();
  //   //ROS_INFO("add edge index:%d size:%d %lf %lf",min_point_index,teb_.sizePoses(), teb_.PoseVertex(min_point_index)->x(),teb_.PoseVertex(min_point_index)->y());
  //   dynobst_edge->setVertex(0,teb_.PoseVertex(min_point_index));
  //   dynobst_edge->setInformation(information);
  //   std::vector<ObstaclePtr> points(2);
  //   points[0]=dynamic_obstacles[min_obstacle_index];
  //   points[1]=dynamic_obstacles[min_obstacle_index+1];
  //   dynobst_edge->setParameters(*cfg_, robot_model_.get(),points);
  //   optimizer_->addEdge(dynobst_edge);
  // }
  if (debug)
    std::cout<<"Edge"<<std::endl;
  for (auto edge:ve)
  {
    if (debug)
    std::cout<<edge.first<<" "<<edge.second<<std::endl;
  }
}

void TebOptimalPlanner::AddEdgesViaPoints()
{
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;
  
  int n = teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;
  //ROS_INFO("via bug");
  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {
    
    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further via-points
     
    // check if point conicides with goal or is located behind it
    if ( index > n-2 ) 
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if ( index < 1)
    {
      if (cfg_->trajectory.via_points_ordered)
      {
        index = 1; // try to connect the via point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
      }
      else
      {
        ROS_DEBUG("TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
        continue; // skip via points really close or behind the current robot pose
      }
    }
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_viapoint);
    
    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0,teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);   
  }
}
void TebOptimalPlanner::AddEdgesGoal()
{
    return;
    //int n = ;
    int index = teb_.sizePoses()-1;
    //double dist = sqrt(pow(teb_.Pose(index).x()-goal_.x(),2)+pow(teb_.Pose(index).y()-goal_.y(),2));
    //ROS_INFO("goal info:%lf %lf dist:%lf",goal_.x(),goal_.y(),dist);
    //if (dist<0.1)
    {
        //goal_ = teb_.BackPose();
        Eigen::Matrix<double,2,2> information;
        information(0,0) = cfg_->optim.weight_goal_point;
        information(1,1) = cfg_->optim.weight_goal_point;
        information(0,1) = 0;
        information(1,0) = 0;
        //Eigen::Vector2d goal = teb_.PoseVertex(index)->position();
        EdgeGoal* edge_goal = new EdgeGoal;
        edge_goal->setVertex(0,teb_.PoseVertex(index));
        edge_goal->setInformation(information);
        edge_goal->setParameters(*cfg_, &goal_);
        optimizer_->addEdge(edge_goal);
    }

}
void TebOptimalPlanner::AddEdgesVelocity()
{
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;
    //ROS_INFO("velocity 1");
    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      //velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
    //ROS_INFO("velocity 2");
    EdgeVelocityGoal* velocity_edge = new EdgeVelocityGoal;

    velocity_edge->setVertex(0,teb_.timeDiffVertex());
    velocity_edge->setVertex(1,teb_.PoseGoal());
    velocity_edge->setVertex(2,teb_.PoseVertex(n-1));

    velocity_edge->setInformation(information);
    velocity_edge->setTebConfig(*cfg_);
    //velocity_edge->setPose(&teb_.Pose(n-1));
    optimizer_->addEdge(velocity_edge);
    //ROS_INFO("velocity 3");
  }
  else // holonomic-robot
  {

    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!
      
    int n = teb_.sizePoses();
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_y;
    information(2,2) = cfg_->optim.weight_max_vel_theta;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      //velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    } 
    
  }
}

void TebOptimalPlanner::AddEdgesAcceleration()
{
  if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0) 
    return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();  
    
  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      //acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      //acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      //acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(1,teb_.PoseGoal());
      acceleration_edge->setVertex(2,teb_.timeDiffVertex());
      //acceleration_edge->setGoalVelocity(&teb_.Pose(n-1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      //acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      //acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      //acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      //acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      //acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(&teb_.Pose(n-1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
}



void TebOptimalPlanner::AddEdgesTimeOptimal()
{
    if (cfg_->optim.weight_optimaltime==0)
      return; // if weight equals zero skip adding edges!

    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_optimaltime);

    for (int i=0; i < teb_.sizePoses()-1; ++i)
    {
      EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
      timeoptimal_edge->setVertex(0,teb_.PoseVertex(i));
      timeoptimal_edge->setVertex(1,teb_.PoseVertex(i+1));
      timeoptimal_edge->setInformation(information);
      timeoptimal_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(timeoptimal_edge);
    }
    EdgeTimeOptimalGoal* timeoptimal_edge = new EdgeTimeOptimalGoal;

    //timeoptimal_edge->setVertex(1,);
    timeoptimal_edge->setVertex(0,teb_.timeDiffVertex());
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
    for (int i=0; i < teb_.sizePoses()-1; ++i)
    {
        EdgeTimeLimit* timelimit_edge = new EdgeTimeLimit;
        timelimit_edge->setVertex(0,teb_.PoseVertex(i));
        timelimit_edge->setVertex(1,teb_.PoseVertex(i+1));
        timelimit_edge->setTebConfig(*cfg_);
        optimizer_->addEdge(timelimit_edge);
    }
  return;
}

void TebOptimalPlanner::AddEdgesShortestPath()
{
    if (cfg_->optim.weight_shortest_path==0)
      return; // if weight equals zero skip adding edges!

    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_shortest_path);

    for (int i=0; i < teb_.sizePoses()-1; ++i)
    {
      EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
      shortest_path_edge->setVertex(0,teb_.PoseVertex(i));
      shortest_path_edge->setVertex(1,teb_.PoseVertex(i+1));
      shortest_path_edge->setInformation(information);
      shortest_path_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(shortest_path_edge);
    }
    EdgeShortestPathGoal* shortest_path_edge = new EdgeShortestPathGoal;
    shortest_path_edge->setVertex(0,teb_.PoseVertex(teb_.sizePoses()-1));
    shortest_path_edge->setVertex(1,teb_.PoseGoal());
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  return;
}
void TebOptimalPlanner::AddEdgesShortestTimeSpace()
{
    if (cfg_->optim.weight_optimaltime==0&&cfg_->optim.weight_shortest_path==0)
           return;
    Eigen::Matrix<double,1,1> information;
    information.fill(1.0);
    for (int i=0; i < teb_.sizePoses()-1; ++i)
    {
      EdgeShortestPath3D* shortest_edge = new EdgeShortestPath3D();
      shortest_edge->setVertex(0,teb_.PoseVertex(i));
      shortest_edge->setVertex(1,teb_.PoseVertex(i+1));
      shortest_edge->setTebConfig(*cfg_);
      shortest_edge->setInformation(information);
      optimizer_->addEdge(shortest_edge);
    }
    EdgeShortestPath3DGoal* shortest_edge = new EdgeShortestPath3DGoal();
    shortest_edge->setVertex(0,teb_.PoseVertex(teb_.sizePoses()-1));
    shortest_edge->setVertex(1,teb_.PoseGoal());
    shortest_edge->setVertex(2,teb_.timeDiffVertex());
    shortest_edge->setTebConfig(*cfg_);
    shortest_edge->setInformation(information);
    optimizer_->addEdge(shortest_edge);
    /*for (int i=0; i < teb_.sizePoses()-1; ++i)
    {
        EdgeTimeLimit* timelimit_edge = new EdgeTimeLimit;
        timelimit_edge->setVertex(0,teb_.PoseVertex(i));
        timelimit_edge->setVertex(1,teb_.PoseVertex(i+1));
        timelimit_edge->setTebConfig(*cfg_);
        optimizer_->addEdge(timelimit_edge);
    }*/
}


void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
      //if (i<teb_.sizePoses()-2)
      //{
          EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
          kinematics_edge->setVertex(0,teb_.PoseVertex(i));
          kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));
          kinematics_edge->setInformation(information_kinematics);
          kinematics_edge->setTebConfig(*cfg_);
          optimizer_->addEdge(kinematics_edge);
     // }else
      //{

      //}
  }	 
  EdgeKinematicsDiffDriveGoal* kinematics_edge = new EdgeKinematicsDiffDriveGoal;
  kinematics_edge->setVertex(0,teb_.PoseVertex(teb_.sizePoses()-1));
  kinematics_edge->setVertex(1,teb_.PoseGoal());
  //kinematics_edge->setPose(&teb_.Pose(i+1));
  kinematics_edge->setInformation(information_kinematics);
  kinematics_edge->setTebConfig(*cfg_);
  optimizer_->addEdge(kinematics_edge);
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }  
}
void TebOptimalPlanner::AddEdgesOnmiDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  //Eigen::Matrix<double,2,2> information_kinematics;
  //information_kinematics.fill(0.0);
  //information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  //information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;

  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeOmniDrive* kinematics_edge = new EdgeOmniDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    //kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));
    //kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    kinematics_edge->setPose(human_pose_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebOptimalPlanner::AddEdgesPreferRotDir()
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    ROS_WARN("TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);
  
  for (int i=0; i < teb_.sizePoses()-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(i+1));      
    rotdir_edge->setInformation(information_rotdir);
    
    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();
    
    optimizer_->addEdge(rotdir_edge);
  }
}

void TebOptimalPlanner::AddEdgesVelocityObstacleRatio()
{
  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(1,1) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(0,1) = information(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  for (int index = 0; index < teb_.sizePoses() - 1; ++index)
  {
    for (const ObstaclePtr obstacle : (*iter_obstacle++))
    {
      EdgeVelocityObstacleRatio* edge = new EdgeVelocityObstacleRatio;
      edge->setVertex(0,teb_.PoseVertex(index));
      edge->setVertex(1,teb_.PoseVertex(index + 1));
      //edge->setVertex(2,teb_.TimeDiffVertex(index));
      edge->setInformation(information);
      edge->setParameters(*cfg_, robot_model_.get(), obstacle.get());
      optimizer_->addEdge(edge);
    }
  }
}

void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{ 
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function 
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();	
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }
  
  optimizer_->computeInitialGuess();
  
  cost_ = 0;

  if (alternative_time_cost)
  {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }
  
  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr)
    {
      cur_cost *= obst_cost_scale;
    }
    else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
    {
      cur_cost *= viapoint_cost_scale;
    }
    else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
    {
      continue; // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag) 
    clearGraph();
}


void TebOptimalPlanner::extractVelocity(const PoseSE3& pose1, const PoseSE3& pose2, double& vx, double& vy, double& omega) const
{
    double dt = pose2.t() - pose1.t();
    if (dt == 0)
    {
        vx = 0;
        vy = 0;
        omega = 0;
        return;
    }
  
    Eigen::Vector2d deltaS = pose2.position() - pose1.position();
  
    if (cfg_->robot.max_vel_y == 0) // nonholonomic robot
    {
      Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
      // translational velocity
      double dir = deltaS.dot(conf1dir);
      vx = (double) g2o::sign(dir) * deltaS.norm()/dt;
      vy = 0;
    }
    else // holonomic robot
    {
      // transform pose 2 into the current robot frame (pose1)
      // for velocities only the rotation of the direction vector is necessary.
      // (map->pose1-frame: inverse 2d rotation matrix)
      double cos_theta1 = std::cos(pose1.theta());
      double sin_theta1 = std::sin(pose1.theta());
      double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
      double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
      vx = p1_dx / dt;
      vy = p1_dy / dt;
    }
  
    // rotational velocity
    double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
    omega = orientdiff/dt;
    // modified by Hu Xingyu
    //ROS_INFO("dx:%lf dy:%lf omega:%lf dt:%lf orientdiff:%lf",deltaS[0],deltaS[1],omega,dt,orientdiff);
    if (cfg_->leadHuman.use_lead&&cfg_->leadHuman.local_pose_to_human_full&&fabs(omega)>cfg_->robot.max_vel_theta)
    {
        vx = 0;
        vy = 0;

        omega = omega<0?-cfg_->robot.max_vel_theta:cfg_->robot.max_vel_theta;
    }
    //ROS_INFO("omega:%lf",omega);
    //end modifiedy by Hu Xingyu
}

bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  if (teb_.sizePoses()<1)
  {
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  look_ahead_poses = std::max(1, std::min(look_ahead_poses, teb_.sizePoses()+1));
  double dt = 0.0;
  for(int counter = 1; counter <= look_ahead_poses; ++counter)
  {
      if (counter<teb_.sizePoses())
      {
          dt += teb_.Pose(counter).t()-teb_.Pose(counter-1).t();
      }else
      {
          dt = teb_.getTimeDiff();
      }

    if(dt >= cfg_->trajectory.dt_ref * look_ahead_poses)  // TODO: change to look-ahead time? Refine trajectory?
    {
        look_ahead_poses = counter + 1;
        break;
    }
  }
  if (dt<=0)
  {	
      for (int i=0;i<teb_.sizePoses();++i)
      {
          ROS_INFO("teb pose:x:%lf y:%lf theta:%lf t:%lf",teb_.Pose(i).x(),teb_.Pose(i).y(),teb_.Pose(i).theta(),teb_.Pose(i).t());
      }
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
	  
  // Get velocity from the first two configurations
  if (look_ahead_poses<teb_.sizePoses())
      extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), vx, vy, omega);
  else
      extractVelocity(teb_.Pose(0), PoseSE3(teb_.getGoal()->pose(),teb_.BackPose().t()+teb_.getTimeDiff()) , vx, vy, omega);
  return true;
}

void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const
{
  int n = teb_.sizePoses();
  velocity_profile.resize( n+2 );

  // start velocity 
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;  
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;
  
  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }
  extractVelocity(teb_.Pose(n-1), PoseSE3(teb_.getGoal()->pose(),teb_.BackPose().t()+teb_.getTimeDiff()), velocity_profile[n].linear.x, velocity_profile[n].linear.y, velocity_profile[n].angular.z);
  
  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;  
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TebOptimalPlanner::getFullTrajectory(std::vector<teb_local_planner_dynamic::TrajectoryPointMsg>& trajectory) const
{
  int n = teb_.sizePoses();
  
  trajectory.resize(n+1);
  
  if (n == 0)
    return;
     
  double curr_time = 0;
  
  // start
  teb_local_planner_dynamic::TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start.fromSec(curr_time);
  
  //curr_time += teb_.TimeDiff(0);
  
  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    teb_local_planner_dynamic::TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
    point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
    point.velocity.angular.z = 0.5*(omega1+omega2);
    curr_time += teb_.Pose(i).t() - teb_.Pose(i-1).t();
    point.time_from_start.fromSec(curr_time);    
  }
  teb_local_planner_dynamic::TrajectoryPointMsg& point = trajectory[n];
  teb_.getGoal()->pose().toPoseMsg(point.pose);
  point.velocity.linear.z = 0;
  point.velocity.angular.x = point.velocity.angular.y = 0;
  double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
  extractVelocity(teb_.Pose(n-2), teb_.Pose(n-1), vel1_x, vel1_y, omega1);
  extractVelocity(teb_.Pose(n-1), PoseSE3(teb_.getGoal()->pose(),teb_.BackPose().t()+teb_.getTimeDiff()), vel2_x, vel2_y, omega2);
  point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
  point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
  point.velocity.angular.z = 0.5*(omega1+omega2);
  curr_time += teb_.getTimeDiff();
  point.time_from_start.fromSec(curr_time);
  
  // goal
  teb_local_planner_dynamic::TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start.fromSec(curr_time);
}


bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses();
  
  for (int i=1; i <= look_ahead_idx; ++i)
  {           
    if ( (i<look_ahead_idx&&costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) == -1)
         ||(i==look_ahead_idx&&costmap_model->footprintCost(teb().PoseGoal()->x(), teb().PoseGoal()->y(), teb().PoseGoal()->theta(), footprint_spec, inscribed_radius, circumscribed_radius) == -1))
    {
      if (visualization_)
      {
          if (i<look_ahead_idx)
              visualization_->publishInfeasibleRobotPose(teb().Pose(i).toPoseSE2(), *robot_model_);
          else
              visualization_->publishInfeasibleRobotPose(teb().getGoal()->pose(), *robot_model_);
      }
      return false;
    }
    // for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
    // {
    //    // ROS_INFO("inner edge dynamic");
    //   if (!(*obst)->isDynamic())
    //     continue;

    //   // Skip first and last pose, as they are fixed
    //   //double time = teb_.TimeDiff(0);
    //   //ROS_INFO("dynamic position:%lf %lf t:%lf",obst->get()->getCentroid()[0],obst->get()->getCentroid()[1],obst->get()->getTime());
    //   //ROS_INFO("local point:%lf %lf t:%lf",teb_.PoseVertex(i)->x(),teb_.PoseVertex(i)->y(),teb_.PoseVertex(i)->t());
    //   double dist;
    //   if (i<teb().sizePoses())
    //     dist = robot_model_->estimateSpatioTemporalDistance(teb_.PoseVertex(i)->pose(), obst->get());
    //   else
    //       dist = robot_model_->estimateSpatioTemporalDistance(PoseSE3(teb_.PoseGoal()->pose(),teb_.BackPose().t()+teb_.getTimeDiff()), obst->get());
    //   //
    //   if for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
    //   {
    //    // ROS_INFO("inner edge dynamic");
    //   if (!(*obst)->isDynamic())
    //     continue;

    //   // Skip first and last pose, as they are fixed
    //   //double time = teb_.TimeDiff(0);
    //   //ROS_INFO("dynamic position:%lf %lf t:%lf",obst->get()->getCentroid()[0],obst->get()->getCentroid()[1],obst->get()->getTime());
    //   //ROS_INFO("local point:%lf %lf t:%lf",teb_.PoseVertex(i)->x(),teb_.PoseVertex(i)->y(),teb_.PoseVertex(i)->t());
    //   double di // ROS_INFO("index:%d dist to dynamic:%lf",i,dist);
    //       // if (i<teb().sizePoses())
    //       // {
    //       //   ROS_INFO("pose:%lf %lf %lf",teb_.PoseVertex(i)->x(),teb_.PoseVertex(i)->y(),teb_.PoseVertex(i)->t());
    //       //   ROS_INFO("obst:%lf %lf %lf",obst->get()->getCentroid3D()[0],obst->get()->getCentroid3D()[1],obst->get()->getCentroid3D()[2]);
    //       // }
    //       // else
    //       // {
    //       //   ROS_INFO("pose:%lf %lf %lf",teb_.PoseGoal()->x(),teb_.PoseGoal()->y(),teb_.BackPose().t()+teb_.getTimeDiff());
    //       //   ROS_INFO("obst:%lf %lf %lf",obst->get()->getCentroid3D()[0],obst->get()->getCentroid3D()[1],obst->get()->getCentroid3D()[2]);
    //       // }
    //       return false;
    //   }

    // }
    //ROS_INFO("end compute");
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i<look_ahead_idx)
    {
        double delta_rot;
        Eigen::Vector2d delta_dist;
        if (i<look_ahead_idx-1)
        {
            delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) -
                                                          g2o::normalize_theta(teb().Pose(i).theta()));
            delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();
        }else
        {
            delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().getGoal()->theta()) -
                                                          g2o::normalize_theta(teb().Pose(i-1).theta()));
            delta_dist = teb().getGoal()->position()-teb().Pose(i-1).position();
        }

      if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
      {
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), 
                                            std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        PoseSE2 intermediate_pose;
        if (i<look_ahead_idx-1)
        {
            intermediate_pose = teb_.Pose(i).toPoseSE2();
        }else
            intermediate_pose = teb_.getGoal()->pose();

        for(int step = 0; step < n_additional_samples; ++step)
        {
          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() + 
                                                           delta_rot / (n_additional_samples + 1.0));
          if ( costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
            footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
          {
            if (visualization_) 
            {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *robot_model_);
            }
            return false;
          }
        }
      }
    }
  }
  return true;
}

} // namespace teb_local_planner
