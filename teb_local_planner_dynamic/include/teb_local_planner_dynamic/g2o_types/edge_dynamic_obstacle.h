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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann, Franz Albers
 *********************************************************************/

#ifndef EDGE_DYNAMICOBSTACLE_H
#define EDGE_DYNAMICOBSTACLE_H

#include <teb_local_planner_dynamic/g2o_types/vertex_pose.h>
#include <teb_local_planner_dynamic/g2o_types/vertex_timediff.h>
#include <teb_local_planner_dynamic/g2o_types/penalties.h>
#include <teb_local_planner_dynamic/g2o_types/base_teb_edges.h>
#include <teb_local_planner_dynamic/obstacles.h>
#include <teb_local_planner_dynamic/teb_config.h>
#include <teb_local_planner_dynamic/robot_footprint_model.h>

namespace teb_local_planner
{
  
/**
 * @class EdgeDynamicObstacle
 * @brief Edge defining the cost function for keeping a distance from dynamic (moving) obstacles.
 * 
 * The edge depends on two vertices \f$ \mathbf{s}_i, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2obstacle) \cdot weight \f$. \n
 * \e dist2obstacle denotes the minimum distance to the obstacle trajectory (spatial and temporal). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow(). \n
 * @see TebOptimalPlanner::AddEdgesDynamicObstacles
 * @remarks Do not forget to call setTebConfig(), setVertexIdx() and 
 * @warning Experimental
 */  
class EdgeDynamicObstacle : public BaseTebUnaryEdge<2, std::vector<ObstaclePtr>, VertexPose>
{
public:
  
  /**
   * @brief Construct edge.
   */    
  EdgeDynamicObstacle()
  {
  }
  
  /**
   * @brief Actual cost function
   */   
  void computeError()
    {
      ROS_ASSERT_MSG(cfg_ && _measurement.size()>0 && robot_model_, "You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeDynamicObstacle()");
      const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);


      auto obst1 = _measurement[0];
      auto obst2 = _measurement[1];
      //if (!bandpt)
          //ROS_INFO("nullptr");
      auto line_start = obst1->getClosestPoint(bandpt->position());
      auto line_end = obst2->getClosestPoint(bandpt->position());
      double scale_t = (bandpt->t()-obst1->getTime())/(obst2->getTime()-obst1->getTime());
      const Eigen::Vector2d line_mid = (line_end-line_start)*scale_t + line_start;
      //ROS_INFO("get here:pose:%lf %lf point:%lf %lf",bandpt->pose().x(),bandpt->pose().y(),line_mid[0],line_mid[1]);
      auto robot_point = robot_model_->getClosestPoint(bandpt->pose(),line_mid);
      Eigen::Vector3d line_start3d(line_start[0],line_start[1],cfg_->optim.weight_dynamic_obstacle_time_factor*obst1->getTime());
      Eigen::Vector3d line_end3d(line_end[0],line_end[1],cfg_->optim.weight_dynamic_obstacle_time_factor*obst2->getTime());
      Eigen::Vector3d robot_point3d(robot_point[0],robot_point[1],cfg_->optim.weight_dynamic_obstacle_time_factor*bandpt->t());
      dv_ = line_end3d-line_start3d;
      dv_.normalize();
      Eigen::Vector3d cross_point;
      dist_ = std::max(1e-6,computeDistPointToLine3d(robot_point3d,line_start3d,line_end3d,&cross_point));
      cross_line_ = robot_point3d-cross_point;
      _error[0] = penaltyBoundFromBelow(dist_, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundFromBelow(dist_, cfg_->obstacles.dynamic_obstacle_inflation_dist, 0.0);
      if (cfg_->optim.obstacle_cost_exponent != 1.0 && cfg_->obstacles.min_obstacle_dist > 0.0)
      {
        // Optional non-linear cost. Note the max cost (before weighting) is
        // the same as the straight line version and that all other costs are
        // below the straight line (for positive exponent), so it may be
        // necessary to increase weight_obstacle and/or the inflation_weight
        // when using larger exponents.
        _error[0] = cfg_->obstacles.min_obstacle_dist * std::pow(_error[0] / cfg_->obstacles.min_obstacle_dist, cfg_->optim.obstacle_cost_exponent);
        _error[1] = cfg_->obstacles.min_obstacle_dist * std::pow(_error[1] / cfg_->obstacles.min_obstacle_dist, cfg_->optim.obstacle_cost_exponent);
      }
      //ROS_INFO("error:%lf",_error[0]).
      ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeDynamicObstacle::computeError() _error[0]=%f\n",_error[0]);
    }
    void linearizeOplus()
    {
        Eigen::Vector3d nx(1.0,0.0,0.0);
        Eigen::Vector3d ny(0.0,1.0,0.0);
        Eigen::Vector3d nz(0.0,0.0,1.0);
        //ROS_INFO("dist is %lf",dist_);
        if (dist_>cfg_->obstacles.min_obstacle_dist)
        {
          _jacobianOplusXi( 0 , 0 ) = 0;
          _jacobianOplusXi( 0 , 1 ) = 0;
          _jacobianOplusXi( 0 , 2 ) = 0;
          _jacobianOplusXi( 0 , 3 ) = 0;
        }else
        {
            _jacobianOplusXi( 0 , 0 ) = cross_line_.dot(nx-dv_[0]*dv_)/dist_;
            _jacobianOplusXi( 0 , 1 ) = cross_line_.dot(ny-dv_[1]*dv_)/dist_;
            _jacobianOplusXi( 0 , 2 ) = cross_line_.dot(nz-dv_[2]*dv_)/dist_;
            _jacobianOplusXi( 0 , 3 ) = 0;
        }
        if (dist_>cfg_->obstacles.dynamic_obstacle_inflation_dist)
        {
          _jacobianOplusXi( 1 , 0 ) = 0;
          _jacobianOplusXi( 1 , 1 ) = 0;
          _jacobianOplusXi( 1 , 2 ) = 0;
          _jacobianOplusXi( 1 , 3 ) = 0;
        }else
        {
            _jacobianOplusXi( 1 , 0 ) = cross_line_.dot(nx-dv_[0]*dv_)/dist_;
            _jacobianOplusXi( 1 , 1 ) = cross_line_.dot(ny-dv_[1]*dv_)/dist_;
            _jacobianOplusXi( 1 , 2 ) = cross_line_.dot(nz-dv_[2]*dv_)/dist_;
            _jacobianOplusXi( 1 , 3 ) = 0;
        }
    }
  
  /**
   * @brief Set Obstacle for the underlying cost function
   * @param obstacle Const pointer to an Obstacle or derived Obstacle
   */     
  void setObstacle(std::vector<ObstaclePtr> obstacle)
  {
    _measurement = obstacle;
  }
  
  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, std::vector<ObstaclePtr> obstacle)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = obstacle;
  }

protected:
  
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  Eigen::Vector3d cross_line_;
    double dist_;
    Eigen::Vector3d dv_;
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
    
 
    

} // end namespace

#endif
