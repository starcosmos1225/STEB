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

#ifndef MISC_H
#define MISC_H

#include <Eigen/Core>
#include <boost/utility.hpp>
#include <boost/type_traits.hpp>
#include <ros/ros.h>

namespace teb_local_planner
{

#define SMALL_NUM 0.00000001

//! Symbols for left/none/right rotations      
enum class RotType { left, none, right };

/** 
 * @brief Check whether two variables (double) are close to each other
 * @param a the first value to compare
 * @param b the second value to compare
 * @param epsilon precision threshold
 * @return \c true if |a-b| < epsilon, false otherwise
 */
inline bool is_close(double a, double b, double epsilon = 1e-4) 
{ 
  return std::fabs(a - b) < epsilon; 
}

/** 
 * @brief Return the average angle of an arbitrary number of given angles [rad]
 * @param angles vector containing all angles
 * @return average / mean angle, that is normalized to [-pi, pi]
 */
inline double average_angles(const std::vector<double>& angles)
{
  double x=0, y=0;
  for (std::vector<double>::const_iterator it = angles.begin(); it!=angles.end(); ++it)
  {
      x += cos(*it);
      y += sin(*it);
  }
  if(x == 0 && y == 0)
      return 0;
  else
      return std::atan2(y, x);
}

/** @brief Small helper function: check if |a|<|b| */
inline bool smaller_than_abs(double i, double j) {return std::fabs(i)<std::fabs(j);}


/**
 * @brief Calculate a fast approximation of a sigmoid function
 * @details The following function is implemented: \f$ x / (1 + |x|) \f$
 * @param x the argument of the function
*/
inline double fast_sigmoid(double x)
{
  return x / (1 + fabs(x));
}

/**
 * @brief Calculate Euclidean distance between two 2D point datatypes
 * @param point1 object containing fields x and y
 * @param point2 object containing fields x and y
 * @return Euclidean distance: ||point2-point1||
*/
template <typename P1, typename P2>
inline double distance_points2d(const P1& point1, const P2& point2)
{
  return std::sqrt( std::pow(point2.x-point1.x,2) + std::pow(point2.y-point1.y,2) );
}


/**
 * @brief Calculate the 2d cross product (returns length of the resulting vector along the z-axis in 3d)
 * @param v1 object containing public methods x() and y()
 * @param v2 object containing fields x() and y()
 * @return magnitude that would result in the 3D case (along the z-axis)
*/
template <typename V1, typename V2>
inline double cross2d(const V1& v1, const V2& v2)
{
     return v1.x()*v2.y() - v2.x()*v1.y();
}

/** 
 * @brief Helper function that returns the const reference to a value defined by either its raw pointer type or const reference.
 * 
 * Return a constant reference for boths input variants (pointer or reference).
 * @remarks Makes only sense in combination with the overload getConstReference(const T& val).
 * @param ptr pointer of type T
 * @tparam T arbitrary type 
 * @return  If \c T is a pointer, return const *T (leading to const T&), otherwise const T& with out pointer-to-ref conversion
 */
template<typename T>
inline const T& get_const_reference(const T* ptr) {return *ptr;}

/** 
 * @brief Helper function that returns the const reference to a value defined by either its raw pointer type or const reference.
 * 
 * Return a constant reference for boths input variants (pointer or reference).
 * @remarks Makes only sense in combination with the overload getConstReference(const T* val).
 * @param val
 * @param dummy SFINAE helper variable
 * @tparam T arbitrary type 
 * @return  If \c T is a pointer, return const *T (leading to const T&), otherwise const T& with out pointer-to-ref conversion
 */
template<typename T>
inline const T& get_const_reference(const T& val, typename boost::disable_if<boost::is_pointer<T> >::type* dummy = 0) {return val;}
// modified by Hu Xingyu
inline void computeLineParam(const Eigen::Vector2d& point1,const Eigen::Vector2d& point2,double& a,double& b,double& c)
{
    double x1 = point1[0];double y1 = point1[1];
    double x2 = point2[0];double y2 = point2[1];
    if (fabs(x2-x1)<1e-10)
    {
        a =1.0;b=0.0;c=-x1;
    }else
    {
        a = (y2-y1);
        b = x1-x2;
        c = y1*x2-y2*x1;
    }
}
inline bool inCircle(const Eigen::Vector2d& center,double radius,const Eigen::Vector2d& point)
{
    double dist = (point-center).norm();
    return dist<radius;
}
inline bool inPolygon(const Eigen::Vector2d& point,const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& vertices)
{
    int n=0;
    for (int i=0;i<vertices.size();i++)
    {
        int j=(i+1)%vertices.size();
        double x=point[0];
        double y=point[1];
        double y_i=vertices[i][1];
        double x_i=vertices[i][0];
        double y_j=vertices[j][1];
        double x_j=vertices[j][0];
        if (y_i>=y&&y_j>=y)
            continue;
        if (y_i<=y&&y_j<=y)
            continue;
//        if (x_j<=x&&(x_i<=x))
//            continue;
        double t = (y-y_i)/(y_j-y_i);
        double x_c = (x_j-x_i)*t+x_i;
        if (x_c<x)
            continue;
        n++;
    }
    //ROS_INFO("cross n:%d",n);
    if (n&1)
        return true;
    return false;
}
inline double computeDistPointToPoint(const Eigen::Vector2d& point1,const Eigen::Vector2d& point2)
{
    return (point1-point2).norm();
}
inline double computeDistPointToLine(const Eigen::Vector2d& point,const Eigen::Vector2d& line1,
                                     const Eigen::Vector2d& line2,Eigen::Vector2d* cross_point=nullptr)
{
    Eigen::Vector2d line_direction = line2-line1;
    line_direction.normalize();
    double t_max;
    Eigen::Vector2d crossPoint;
    if (line_direction[0]>1e-5)
        t_max = (line2-line1)[0]/line_direction[0];
    else
        t_max = (line2-line1)[1]/line_direction[1];
    Eigen::Vector2d orthogonal_direction(-line_direction[1],line_direction[0]);
    double A1,A2,B1,B2,C1,C2;
    computeLineParam(line1,line2,A1,B1,C1);
    computeLineParam(point,point+t_max*orthogonal_direction,A2,B2,C2);
    double D = A1*B2-A2*B1;
    crossPoint[0] = (B1*C2-B2*C1)/D;
    crossPoint[1] = (A2*C1-A1*C2)/D;
    double t_current =0;
    if (line_direction[0]>1e-5)
        t_current = (crossPoint-line1)[0]/line_direction[0];
    else
        t_current = (crossPoint-line1)[1]/line_direction[1];
    if (t_current>=0&&t_current<=t_max)
    {
        //ROS_INFO("for debug:return in the line");
        (*cross_point) = crossPoint;
        return (crossPoint-point).norm();

    }else
    {
        //ROS_INFO("for debug:return out of the line");
        double dist1 = (line1-point).norm();
        double dist2 = (line2-point).norm();
        if (dist1<dist2)
        {
            (*cross_point)=line1;
            return dist1;
        }
        else
        {
            (*cross_point)=line2;
            return dist2;
        }
    }


}
inline double computeDistPointToLine3d(const Eigen::Vector3d& point,const Eigen::Vector3d& line1,
                                     const Eigen::Vector3d& line2,Eigen::Vector3d* cross_point=nullptr)
{
    Eigen::Vector3d line_direction = line2-line1;
    line_direction.normalize();
    Eigen::Vector3d point_line1 = point-line1;
    double t = line_direction.dot(point_line1)/(line_direction.squaredNorm());
    Eigen::Vector3d crossPoint;
    crossPoint = line1+t*line_direction;
    if (cross_point)
        (*cross_point) = crossPoint;
    return (crossPoint-point).norm();
}
inline double computeDistPointToLine3dQuadratic(const Eigen::Vector3d& point,const Eigen::Vector3d& line1,
                                     const Eigen::Vector3d& line2,Eigen::Vector3d* cross_point=nullptr)
{
    Eigen::Vector3d line_direction = line2-line1;
    line_direction.normalize();
    Eigen::Vector3d point_line1 = point-line1;
    double t = line_direction.dot(point_line1)/(line_direction.squaredNorm());
    Eigen::Vector3d crossPoint;
    crossPoint = line1+t*line_direction;
    double dt2 = 1.0*pow(crossPoint[2]-point[2],2);
    double time_dist_2 = dt2*dt2;
    double space_dist_2 = pow(crossPoint[0]-point[0],2)+pow(crossPoint[1]-point[1],2);
    if (cross_point)
        (*cross_point) = crossPoint;
    return sqrt(space_dist_2+time_dist_2);
}
inline double computeNearestPoint(const Eigen::Vector2d& center,double radius,
                                  const Eigen::Vector2d& point1,const Eigen::Vector2d& point2,
                                  Eigen::Vector2d& circle_point)
{
    bool check_point_1_inCircle = inCircle(center,radius,point1);
    bool check_point_2_inCircle = inCircle(center,radius,point2);
    if(check_point_1_inCircle&&check_point_2_inCircle)
    {
        // two points in the circle
        //ROS_INFO("for debug:return inner");
        return std::numeric_limits<double>::max();
    }
    Eigen::Vector2d line_direction = point2-point1;
    line_direction.normalize();
    Eigen::Vector2d orthogonal_direction(-line_direction[1],line_direction[0]);
    Eigen::Vector2d crossPoint;
    double t_max;
    if (fabs(line_direction[0])>1e-5)
        t_max = (point2-point1)[0]/line_direction[0];
    else
        t_max = (point2-point1)[1]/line_direction[1];
    if (check_point_1_inCircle||check_point_2_inCircle)
    {
        //only one point in the circle, so it must cross the circle with one cross point;
        //ROS_INFO("for debug:return one cross");
        double d1 = point1[0] - center[0];
        double d2 = point1[1] - center[1];
        double A = line_direction.squaredNorm();
        double B = 2*(line_direction[0]*d1+line_direction[1]*d2);
        double C = d1*d1+d2*d2-radius*radius;
        double t_current;
        if (check_point_1_inCircle)
        {
            t_current = (-B+sqrt(B*B-4*A*C))*0.5/A;
        }else
        {
            t_current = (-B-sqrt(B*B-4*A*C))*0.5/A;
        }
        //ROS_INFO("for debug:point1:%lf %lf line_direction:%lf %lf A: %lf t_current: %lf",point1[0],point1[1],
                //line_direction[0],line_direction[1],A,t_current);
        circle_point = point1+t_current*line_direction;

        return 0.0;
    }
    double A1,A2,B1,B2,C1,C2;
    computeLineParam(point1,point2,A1,B1,C1);
    computeLineParam(center,center+t_max*orthogonal_direction,A2,B2,C2);
    double D = A1*B2-A2*B1;
    /*
     * a1a2x+b1a2y+c1a2 = 0
     * a2a1x+b2a1y+c2a1 = 0
     * (b1a2-b2a1)y+ (c1a2-c2a1) = 0
     * (a1b2-a2b1)x + (c1b2-c2b1) = 0
     * y = (c1a2-c2a1)/(b2a1-b1a2)
     * x = (c2b1-c1b2)/(b2a1-b1a2)
     */
    crossPoint[0] = (B1*C2-B2*C1)/D;
    crossPoint[1] = (A2*C1-A1*C2)/D;
    //ROS_INFO("point1:%lf %lf point2:%lf %lf cross point:%lf %lf center:%lf %lf",point1[0],point1[1],
            //point2[0],point2[1],crossPoint[0],crossPoint[1],center[0],center[1]);
    double t_current =0;
    if (fabs(line_direction[0])>1e-5)
        t_current = (crossPoint-point1)[0]/line_direction[0];
    else
        t_current = (crossPoint-point1)[1]/line_direction[1];
    //ROS_INFO("tmax:%lf tmin:0 t_current:%lf crossPoint:%lf %lf point1:%lf %lf point2:%lf %lf center:%lf %lf",
             //t_max,t_current,crossPoint[0],crossPoint[1],point1[0],point1[1],point2[0],point2[1],center[0],center[1]);
    if (t_current>=0&&t_current<=t_max)
    {

        double min_distance = (crossPoint-center).norm()-radius;
        //ROS_INFO("for debug:return in the line min dist: %lf",min_distance);
        orthogonal_direction = crossPoint-center;
        orthogonal_direction.normalize();
        if (min_distance>=0)
        {
            circle_point = center + radius*orthogonal_direction;
            return min_distance;
        }
        else
        {
            double step = min_distance*(min_distance + 2*radius);
            circle_point = crossPoint-step*line_direction;
            return 0.0;
        }
    }else
    {
        //ROS_INFO("for debug:return out of the line");
        double dist1 = (point1-center).norm();
        double dist2 = (point2-center).norm();
        if (dist1<dist2)
        {
            line_direction = point1-center;
            line_direction.normalize();
            circle_point = center+ radius*line_direction;
            //ROS_INFO("for debug:return out of the line %lf",dist1-radius);
            return dist1-radius;
        }else
        {
            line_direction = point2-center;
            line_direction.normalize();
            circle_point = center+ radius*line_direction;
            //ROS_INFO("for debug:return out of the line %lf",dist2-radius);
            return dist2-radius;
        }
    }
}

// end modified by Hu Xingyu
} // namespace teb_local_planner

#endif /* MISC_H */
