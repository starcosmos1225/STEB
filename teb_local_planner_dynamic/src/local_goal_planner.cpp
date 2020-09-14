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
 * Author: Hu Xingyu
 *********************************************************************/
// modified by HU Xingyu
#include <teb_local_planner_dynamic/local_goal_planner.h>

// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner_dynamic/g2o_types/vertex_pose.h>
#include <teb_local_planner_dynamic/g2o_types/edge_local_obstacle.h>
#include <teb_local_planner_dynamic/g2o_types/edge_plan.h>
#include <teb_local_planner_dynamic/g2o_types/edge_leadhuman.h>

#include <memory>
#include <limits>


namespace teb_local_planner
{

// ============== Implementation ===================

TebLocalGoalPlanner::TebLocalGoalPlanner() : cfg_(NULL), obstacles_(NULL), cost_(HUGE_VAL),
    robot_model_(new PointRobotFootprint()), initialized_(false), optimized_(false),local_goal_(NULL)
{    
}
  
TebLocalGoalPlanner::TebLocalGoalPlanner(const TebConfig* cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model):
    cfg_(NULL), obstacles_(NULL), cost_(HUGE_VAL),
    robot_model_(new PointRobotFootprint()), initialized_(false), optimized_(false),local_goal_(NULL)
{    
  initialize(cfg, obstacles, robot_model);
}

TebLocalGoalPlanner::~TebLocalGoalPlanner()
{
  clearGraph();
}

void TebLocalGoalPlanner::initialize(const TebConfig* cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model)
{    
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();
  
  cfg_ = cfg;
  obstacles_ = obstacles;
  robot_model_ = robot_model;
  cost_ = HUGE_VAL;
  initialized_ = true;
}
/*
 * registers custom vertices and edges in g2o framework
 */

void TebLocalGoalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  //factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("EDGE_LOCAL_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeLocalObstacle>);
  factory->registerType("EDGE_PLAN_OPTIMAL", new g2o::HyperGraphElementCreator<EdgePlan>);
  factory->registerType("EDGE_LEAD_HUMAN", new g2o::HyperGraphElementCreator<EdgeLeadHuman>);
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
boost::shared_ptr<g2o::SparseOptimizer> TebLocalGoalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebLocalGoalPlanner instances (thread-safe)
  static boost::once_flag flag_local_goal = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypes, flag_local_goal);

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


bool TebLocalGoalPlanner::optimizeTEB(int iterations_innerloop,
                                      bool compute_cost_afterwards,
                                      double obst_cost_scale)
{
  if (cfg_->optim.optimization_activate==false) 
    return false;
  
  bool success = false;
  optimized_ = false;
  
  double weight_multiplier = 1.0;
  

  success = buildGraph(weight_multiplier);

  if (!success || !obstacles_nearby_)
  {
      clearGraph();
      return false;
  }
  success = optimizeGraph(iterations_innerloop, false);
  if (!success)
  {
      clearGraph();
      return false;
  }
  optimized_ = true;
    
  if (compute_cost_afterwards) // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale);
  clearGraph();
  return true;
}

bool TebLocalGoalPlanner::plan(std::vector<geometry_msgs::PoseStamped>* global_plan, const geometry_msgs::PoseStamped& human_pose)
{    
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  human_pose_ = human_pose;
  global_plan_ = global_plan;
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations);
}


bool TebLocalGoalPlanner::buildGraph(double weight_multiplier)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }
  
  // add  vertices
  AddVertices();
  
  // add Edges (local cost functions)
  AddEdgesObstacle(weight_multiplier);
  AddEdgesHumanPose();
  AddEdgesPlan();
  return true;  
}

bool TebLocalGoalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  return true;
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);
  if(!iter)
  {
	ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }
  if (clear_after) clearGraph();	
  return true;
}

void TebLocalGoalPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    //optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
    optimizer_->vertices().clear();  // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}



void TebLocalGoalPlanner::AddVertices()
{
  // add vertices to graph
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding Local vertice ...");
  local_goal_ = new VertexPose();
  local_goal_->setId(0);
  // compute a init value
  computeInitPose(human_pose_,global_plan_);
  optimizer_->addVertex(local_goal_);

}

void TebLocalGoalPlanner::AddEdgesObstacle(double weight_multiplier)
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

  obstacles_with_vertex.clear();

  auto create_edge = [inflated, &information, &information_inflated, this] (const Obstacle* obstacle) {
    if (inflated)
    {
      EdgeLocalInflatedObstacle* dist_bandpt_obst = new EdgeLocalInflatedObstacle;
      dist_bandpt_obst->setVertex(0,local_goal_);
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
      EdgeLocalObstacle* dist_bandpt_obst = new EdgeLocalObstacle;
      dist_bandpt_obst->setVertex(0,local_goal_);
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    };
  };
    

  double left_min_dist = std::numeric_limits<double>::max();
  double right_min_dist = std::numeric_limits<double>::max();
  ObstaclePtr left_obstacle;
  ObstaclePtr right_obstacle;
  for (const ObstaclePtr& obst : *obstacles_)
  {
    // we handle dynamic obstacles differently below
    if(cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
        continue;

      // calculate distance to robot model
    double dist = robot_model_->calculateDistance(local_goal_->pose().toPoseSE2(), obst.get());
          
      // force considering obstacle if really close to the current pose
    if (dist < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor)
    {
        obstacles_with_vertex.push_back(obst);
        continue;
    }
    // cut-off distance
    if (dist > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor)
        continue;
          
    // determine side (left or right) and assign obstacle if closer than the previous one
    const Eigen::Vector2d pose_orient = Eigen::Vector2d(cos(local_goal_->theta()),sin(local_goal_->theta()));
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
      obstacles_with_vertex.push_back(left_obstacle);
  if (right_obstacle)
      obstacles_with_vertex.push_back(right_obstacle);
  // create obstacle edges
  // compute a min_dist to find whether there is any obstacles that effect the local_goal
  double min_dist = left_min_dist<right_min_dist?left_min_dist:right_min_dist;
  if (min_dist<cfg_->leadHuman.min_obstacle_dist)
  {
      obstacles_nearby_ = true;
  }else
      obstacles_nearby_ = false;
  for (const ObstaclePtr obst : obstacles_with_vertex)
      create_edge(obst.get());
}
void TebLocalGoalPlanner::AddEdgesHumanPose()
{
    EdgeLeadHuman* edge_lead_human = new EdgeLeadHuman;
    edge_lead_human->setVertex(0,local_goal_);
    edge_lead_human->setParameters(*cfg_,&human_pose_);
    optimizer_->addEdge(edge_lead_human);
}
void TebLocalGoalPlanner::AddEdgesPlan()
{
    for (unsigned int i=min_point_index_;i<(*global_plan_).size()-1&&i<(unsigned int)(min_point_index_+cfg_->leadHuman.nearestK);++i)
    {
        EdgePlan* edge_plan = new EdgePlan;
        edge_plan->setVertex(0,local_goal_);
        std::vector<geometry_msgs::PoseStamped* > plan({&(global_plan_->at(i)),&(global_plan_->at(i+1))});
        edge_plan->setParameters(*cfg_,plan);
        optimizer_->addEdge(edge_plan);
    }
}

void TebLocalGoalPlanner::computeCurrentCost(double obst_cost_scale)
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
  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeLocalObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeLocalInflatedObstacle*>(*it) != nullptr)
    {
      cur_cost *= obst_cost_scale;
    }
    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag) 
    clearGraph();
}
PoseSE3 TebLocalGoalPlanner::getOptimalGoal()
{
    ROS_ASSERT_MSG(optimized_, "Call optimizer() first.");
    return local_goal_->pose();
}
void TebLocalGoalPlanner::computeInitPose(geometry_msgs::PoseStamped& constrain_pose,
                                                const std::vector<geometry_msgs::PoseStamped>* global_plan)
{
    if (global_plan->size()==0)
    {
        ROS_ERROR("TebLocalGoalPlanner: the global plan is empty!");
        return;
    }
    Eigen::Vector2d human_position(constrain_pose.pose.position.x,constrain_pose.pose.position.y);
    if (global_plan->size()==1)
    {
        double dist = sqrt(pow(global_plan->at(0).pose.position.x - constrain_pose.pose.position.x,2)+
                           pow(global_plan->at(0).pose.position.y - constrain_pose.pose.position.y,2));
        if (dist<cfg_->leadHuman.max_dist)
        {
            local_goal_->x() = global_plan->at(0).pose.position.x;
            local_goal_->y() = global_plan->at(0).pose.position.y;
            //if (cfg_->leadHuman.local_pose_to_human)
            //{
            //Eigen::Vector2d local_goal_2_human = local_goal_->position() - human_position;
            //local_goal_->theta() = std::atan2(local_goal_2_human[1],local_goal_2_human[0]);
            //}
            return;
        }
        else
        {
            // compute the cross point between circle and line
            Eigen::Vector2d position(constrain_pose.pose.position.x,constrain_pose.pose.position.y);
            Eigen::Vector2d goal(global_plan->at(0).pose.position.x,global_plan->at(0).pose.position.y);
            Eigen::Vector2d direction = goal-position;
            direction.normalize();
            goal = position + cfg_->leadHuman.max_dist*direction;
            local_goal_->position() = goal;
            //if (cfg_->leadHuman.local_pose_to_human)
            //{
                //Eigen::Vector2d local_goal_2_human = human_position-local_goal_->position();
                //local_goal_->theta() = std::atan2(local_goal_2_human[1],local_goal_2_human[0]);
            //}
            return;
        }
    }

    Eigen::Vector2d center(constrain_pose.pose.position.x,constrain_pose.pose.position.y);
    Eigen::Vector2d minPoint;
    double minDist = std::numeric_limits<double>::max();
    // find a nearest point to global_plan lines in the human max_dist circle.
    for (unsigned int i=0;i<global_plan->size()-1;++i)
    {
        Eigen::Vector2d circle_point;
        Eigen::Vector2d point1(global_plan->at(i).pose.position.x,global_plan->at(i).pose.position.y);
        Eigen::Vector2d point2(global_plan->at(i+1).pose.position.x,global_plan->at(i+1).pose.position.y);

        double dist = computeNearestPoint(center,cfg_->leadHuman.max_dist,point1,point2,circle_point);
        if (dist<=minDist+1e-6)
        {
            minDist = dist;
            minPoint = circle_point;
            min_point_index_ = i;
        }
    }
    local_goal_->position() = minPoint;
    //if (cfg_->leadHuman.local_pose_to_human)
    //{
    //Eigen::Vector2d local_goal_2_human = local_goal_->position() - human_position;
    //local_goal_->theta() = std::atan2(local_goal_2_human[1],local_goal_2_human[0]);
    //}

}

} // namespace teb_local_planner
// end modified by HU Xingyu
