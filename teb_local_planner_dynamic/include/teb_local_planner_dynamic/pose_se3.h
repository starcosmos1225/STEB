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

#ifndef POSE_SE3_H_
#define POSE_SE3_H_

#include <g2o/stuff/misc.h>

#include <Eigen/Core>
#include <teb_local_planner_dynamic/misc.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <teb_local_planner_dynamic/pose_se2.h>
namespace teb_local_planner
{

class PoseSE3
{
public:
    
  PoseSE3()
  {
    setZero();
  }
      
  PoseSE3(const Eigen::Ref<const Eigen::Vector2d>& position, double t,double theta)
  {
      _position = position;
      _time = t;
      _theta = theta;
  }
  
  PoseSE3(double x, double y, double t,double theta)
  {
      _position.coeffRef(0) = x;
      _position.coeffRef(1) = y;
      _time = t;
      _theta = theta;
  }
  
  PoseSE3(const geometry_msgs::Pose& pose,double t)
  {
      _position.coeffRef(0) = pose.position.x;
      _position.coeffRef(1) = pose.position.y;
      _time = t;
      _theta = tf::getYaw( pose.orientation );
  }
  
  PoseSE3(const tf::Pose& pose, double t)
  {
      _position.coeffRef(0) = pose.getOrigin().getX();
      _position.coeffRef(1) = pose.getOrigin().getY();
      _time = t;
      _theta = tf::getYaw( pose.getRotation() );
  }
  
  PoseSE3(const PoseSE2& pose, double t)
  {
      _position = pose.position();
      _time = t;
      _theta = pose.theta();
  }
  PoseSE3(const PoseSE3& pose)
  {
      _position = pose._position;
      _time = pose._time;
      _theta = pose._theta;
  }
	
  ~PoseSE3() {}
  
  Eigen::Vector2d& position() {return _position;}
  const Eigen::Vector2d& position() const {return _position;}
  double& x() {return _position.coeffRef(0);}
  const double& x() const {return _position.coeffRef(0);}
  double& y() {return _position.coeffRef(1);}
  const double& y() const {return _position.coeffRef(1);}
  double& t() {return _time;}
  const double& t() const {return _time;}
  double& theta() {return _theta;}
  const double& theta() const {return _theta;}
  void setZero()
  {
    _position.setZero();
    _time = 0;
    _theta = 0;
  }
  void toPoseMsg(geometry_msgs::Pose& pose) const
  {
    pose.position.x = _position.x();
    pose.position.y = _position.y();
    pose.position.z = 0;
    pose.orientation = tf::createQuaternionMsgFromYaw(_theta);
  }

  Eigen::Vector2d orientationUnitVec() const {return Eigen::Vector2d(std::cos(_theta), std::sin(_theta));}
  void scale(double factor)
  {
    _position *= factor;
    _time *= factor;
    _theta = g2o::normalize_theta( _theta*factor );
  }
  void plus(const double* pose_as_array)
  {
    _position.coeffRef(0) += pose_as_array[0];
    _position.coeffRef(1) += pose_as_array[1];
    _time += pose_as_array[2];
    _theta = g2o::normalize_theta( _theta + pose_as_array[3] );
  }
  PoseSE2 toPoseSE2()
  {
      return PoseSE2(_position,_theta);
  }
  const PoseSE2 toPoseSE2() const
  {
      return PoseSE2(_position,_theta);
  }
  /**
    * @brief Get the mean / average of two poses and store it in the caller class
    * For the position part: 0.5*(x1+x2)
    * For the angle: take the angle of the mean direction vector
    * @param pose1 first pose to consider
    * @param pose2 second pose to consider
    */ 
  void averageInPlace(const PoseSE3& pose1, const PoseSE3& pose2)
  {
    _position = (pose1._position + pose2._position)*0.5;
    _time = (pose1._time + pose2._time)*0.5;
    _theta = g2o::average_angle(pose1._theta, pose2._theta);
  }
  
  /**
    * @brief Get the mean / average of two poses and return the result (static)
    * For the position part: 0.5*(x1+x2)
    * For the angle: take the angle of the mean direction vector
    * @param pose1 first pose to consider
    * @param pose2 second pose to consider
    * @return mean / average of \c pose1 and \c pose2
    */ 
  static PoseSE3 average(const PoseSE3& pose1, const PoseSE3& pose2)
  {
    return PoseSE3( (pose1._position + pose2._position)*0.5 , (pose1._time + pose2._time)*0.5,g2o::average_angle(pose1._theta, pose2._theta) );
  }
  
  /**
    * @brief Rotate pose globally
    * 
    * Compute [pose_x, pose_y] = Rot(\c angle) * [pose_x, pose_y].
    * if \c adjust_theta, pose_theta is also rotated by \c angle
    * @param angle the angle defining the 2d rotation
    * @param adjust_theta if \c true, the orientation theta is also rotated
    */ 
  void rotateGlobal(double angle, bool adjust_theta=true)
  {
    double new_x = std::cos(angle)*_position.x() - std::sin(angle)*_position.y();
    double new_y = std::sin(angle)*_position.x() + std::cos(angle)*_position.y();
    _position.x() = new_x;
    _position.y() = new_y;
    if (adjust_theta)
        _theta = g2o::normalize_theta(_theta+angle);
  }
  
  ///@}
  
  
  /** @name Operator overloads / Allow some arithmetic operations */
  ///@{ 
  
  /**
    * @brief Asignment operator
    * @param rhs PoseSE2 instance
    * @todo exception safe version of the assignment operator
    */ 	
  PoseSE3& operator=( const PoseSE3& rhs )
  {
    if (&rhs != this)
    {
	_position = rhs._position;
    _time = rhs._time;
	_theta = rhs._theta;
    }
    return *this;
  }

  /**
    * @brief Compound assignment operator (addition)
    * @param rhs addend
    */ 
  PoseSE3& operator+=(const PoseSE3& rhs)
  {
    _position += rhs._position;
    _time += rhs._time;
    _theta = g2o::normalize_theta(_theta + rhs._theta);
    return *this;
  }
  
  /**
  * @brief Arithmetic operator overload for additions
  * @param lhs First addend
  * @param rhs Second addend
  */ 
  friend PoseSE3 operator+(PoseSE3 lhs, const PoseSE3& rhs)
  {
    return lhs += rhs;
  }
  
  /**
    * @brief Compound assignment operator (subtraction)
    * @param rhs value to subtract
    */ 
  PoseSE3& operator-=(const PoseSE3& rhs)
  {
    _position -= rhs._position;
    _time -= rhs._time;
    _theta = g2o::normalize_theta(_theta - rhs._theta);
    return *this;
  }
  

  friend PoseSE3 operator-(PoseSE3 lhs, const PoseSE3& rhs)
  {
    return lhs -= rhs;
  }
  friend PoseSE3 operator*(PoseSE3 pose, double scalar)
  {
    pose._position *= scalar;
    pose._time *= scalar;
    pose._theta *= scalar;
    return pose;
  }
  friend PoseSE3 operator*(double scalar, PoseSE3 pose)
  {
    pose._position *= scalar;
    pose._time *= scalar;
    pose._theta *= scalar;
    return pose;
  }
  
    friend std::ostream& operator<< (std::ostream& stream, const PoseSE3& pose)
	{
        stream << "x: " << pose._position[0] << " y: " << pose._position[1] <<" time: " << pose._time << " theta: " << pose._theta;
    return stream;
	}
  
  ///@}
      
      
private:
  
  Eigen::Vector2d _position;
  double _time;
  double _theta;
      
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
};


} // namespace teb_local_planner

#endif // POSE_SE3_H_
