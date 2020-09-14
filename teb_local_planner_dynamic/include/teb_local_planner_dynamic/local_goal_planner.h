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
// modified by HU Xingyu
#ifndef TEB_LOCAL_GOAL_PLANNER_H_1
#define TEB_LOCAL_GOAL_PLANNER_H_1

#include <math.h>


// teb stuff
#include <teb_local_planner_dynamic/teb_config.h>
#include <teb_local_planner_dynamic/misc.h>
#include <teb_local_planner_dynamic/robot_footprint_model.h>
#include <teb_local_planner_dynamic/g2o_types/vertex_pose.h>

// g2o lib stuff
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

// messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>


#include <nav_msgs/Odometry.h>
#include <limits.h>

// boost
#include <boost/thread.hpp>

namespace teb_local_planner
{
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  TEBBlockSolver;

//! Typedef for the linear solver utilized for optimization
typedef g2o::LinearSolverCSparse<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;
class TebLocalGoalPlanner
{
public:
    /**
     * @brief TebLocalGoalPlanner the default constructor
     */
    TebLocalGoalPlanner();
    /**
     * @brief TebLocalGoalPlanner the constructor we use
     * @param cfg set the config to planner
     * @param obstacles a pointer of obstacles
     * @param robot_model the robot's footprint model
     */
    TebLocalGoalPlanner(const TebConfig* cfg, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>());
    /**
     * @brief ~TebLocalGoalPlanner the default destructor
     */
    ~TebLocalGoalPlanner();
    /**
     * @brief initialize init the varible
     * @param cfg the config to planner
     * @param obstacles a pointer of obstacles
     * @param robot_model the robot's footprint model
     */
    void initialize(const TebConfig* cfg, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>());
    /**
     * @brief plan plan a local goal
     * @param global_plan the global plan come from teb_local_planner_ros
     * @param human_pose the constrained human pose
     * @return true if plan success, otherwise false
     */
    bool plan(std::vector<geometry_msgs::PoseStamped>* global_plan, const geometry_msgs::PoseStamped& human_pose);
    /**
     * @brief getOptimalGoal return the optimized goal
     * @return local goal
     */
    PoseSE3 getOptimalGoal();
    /**
     * @brief optimizeTEB optimize the local goal
     * @param iterations_innerloop the g2o loop iterations
     * @param compute_cost_afterwards: no use
     * @param obst_cost_scale: the cost weight
     * @return true if optimize success by g2o, otherwise false
     */
    bool optimizeTEB(int iterations_innerloop, bool compute_cost_afterwards = false,
                     double obst_cost_scale=1.0);
    /**
     * @brief setObstVector set obstacles
     * @param obst_vector a pointer of obstacles' containner
     */
    void setObstVector(ObstContainer* obst_vector) {obstacles_ = obst_vector;}
    /**
     * @brief getObstVector get the containner of obstacles
     * @return the containner of obstacles
     */
    ObstContainer& getObstVector() const {return *obstacles_;}
    /**
     * @brief registerG2OTypes g2o method to regist the edge and vertex types
     */
    static void registerG2OTypes();
    /**
     * @brief optimizer get the optimizer
     * @return a pointer to g20::optimizer
     */
    boost::shared_ptr<g2o::SparseOptimizer> optimizer() {return optimizer_;}
    /**
     * @brief optimizer const method for getting optimizer
     * @return a const pointer to g20::optimizer
     */
    boost::shared_ptr<const g2o::SparseOptimizer> optimizer() const {return optimizer_;}
    /**
     * @brief isOptimized check whether optimized
     * @return true if optimized, otherwise false
     */
    bool isOptimized() const {return optimized_;}
    /**
     * @brief computeCurrentCost get the optimized cost
     * @param obst_cost_scale the obstacle's weight
     */
    void computeCurrentCost(double obst_cost_scale=1.0);
    /**
     * @brief computeCurrentCost another method for compute cost
     * @param cost[in, out]: get the cost for each vertex
     * @param obst_cost_scale the obstacle's weight
     */
    void computeCurrentCost(std::vector<double>& cost, double obst_cost_scale=1.0)
    {
      computeCurrentCost(obst_cost_scale);
      cost.push_back( getCurrentCost() );
    }
    /**
     * @brief getCurrentCost get the computed cost
     * @return the cost
     */
    double getCurrentCost() const {return cost_;}
protected:
    /**
     * @brief buildGraph g2o method to build the optimizer graph
     * @param weight_multiplier no use
     * @return true if build success, otherwise false
     */
    bool buildGraph(double weight_multiplier=1.0);
    /**
     * @brief optimizeGraph optimize the graph
     * @param no_iterations the iteration number
     * @param clear_after whether to clear the graph after optimized
     * @return true if optimize success, otherwise false
     */
    bool optimizeGraph(int no_iterations, bool clear_after=true);
    /**
     * @brief clearGraph clear the g2o graph
     */
    void clearGraph();
    /**
     * @brief computeInitPose compute the init local goal for optimize. This method is purely geometric.
     * @param constrain_pose the constrained human pose
     * @param global_plan the global plan come from teb_local_planner_ros
     */
    void computeInitPose(geometry_msgs::PoseStamped& constrain_pose,const std::vector<geometry_msgs::PoseStamped>* global_plan);
    /**
     * @brief AddVertices add vertices into g2o graph
     */
    void AddVertices();
    /**
     * @brief AddEdgesObstacle add obstacle edges into g2o graph
     * @param weight_multiplier no use
     */
    void AddEdgesObstacle(double weight_multiplier=1.0);
    /**
     * @brief AddEdgesHumanPose add humanpose edges into g2o graph
     */
    void AddEdgesHumanPose();
    /**
     * @brief AddEdgesPlan add plan edges into g2o graph
     */
    void AddEdgesPlan();
    /**
     * @brief initOptimizer create a optimizer
     * @return g2o optimizer
     */
    boost::shared_ptr<g2o::SparseOptimizer> initOptimizer();

    // Member variables

    std::vector<geometry_msgs::PoseStamped>* global_plan_;//! <A pointer to global plan
    const TebConfig* cfg_; //!< Config class that stores and manages all related parameters
    ObstContainer* obstacles_; //!< Store obstacles that are relevant for planning
    ObstContainer obstacles_with_vertex; //! < the obstacles that will effect the local goal
    double cost_; //!< Store cost value of the current hyper-graph
    RobotFootprintModelPtr robot_model_; //!< Robot model
    boost::shared_ptr<g2o::SparseOptimizer> optimizer_; //!< g2o optimizer for trajectory optimization
    bool initialized_; //!< Keeps track about the correct initialization of this class
    bool optimized_; //!< This variable is \c true as long as the last optimization has been completed successful
    geometry_msgs::PoseStamped human_pose_;//!< the human's position and pose
    VertexPose* local_goal_;//!< the local goal which we need to optimize
    int min_point_index_; //! < the global plan's min index that will effect the local goal
    bool obstacles_nearby_;//! <wheter the obstacles in the vertex's nearby. That means the obstacels could constratin
                           //! <the position of vertex. If not, we need not optimal the vertex because the init position
                           //! <is the best one.
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
};

//! Abbrev. for shared instances of the TebOptimalPlanner
typedef boost::shared_ptr<TebLocalGoalPlanner> TebLocalGoalPlannerPtr;
//! Abbrev. for shared const TebOptimalPlanner pointers
typedef boost::shared_ptr<const TebLocalGoalPlanner> TebLocalGoalPlannerConstPtr;
//! Abbrev. for containers storing multiple teb optimal planners
typedef std::vector< TebLocalGoalPlannerPtr > TebLocalGoalPlannerContainer;

} // namespace teb_local_planner

#endif /* OPTIMAL_PLANNER_H_ */
// end modified by HU Xingyu
