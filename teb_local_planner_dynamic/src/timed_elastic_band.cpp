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

#include <teb_local_planner_dynamic/timed_elastic_band.h>

#include <limits>

namespace teb_local_planner
{

namespace
{
  /**
   * estimate the time to move from start to end.
   * Assumes constant velocity for the motion.
   */
  double estimateDeltaT(const PoseSE3& start, const PoseSE2& end,
                        double max_vel_x, double max_vel_theta)
  {
      double dt_constant_motion = 0.1;
      if (max_vel_x > 0) {
        double trans_dist = (end.position() - start.position()).norm();
        dt_constant_motion = trans_dist / max_vel_x;
      }
      if (max_vel_theta > 0) {
        double rot_dist = std::abs(g2o::normalize_theta(end.theta() - start.theta()));
        dt_constant_motion = std::max(dt_constant_motion, rot_dist / max_vel_theta);
      }
      return dt_constant_motion;
  }

} // namespace


TimedElasticBand::TimedElasticBand():time_scale_(1.0)
{}
TimedElasticBand::TimedElasticBand(double time_scale):time_scale_(time_scale)
{}
TimedElasticBand::~TimedElasticBand()
{
  ROS_DEBUG("Destructor Timed_Elastic_Band...");
  clearTimedElasticBand();
}


void TimedElasticBand::addPose(const PoseSE3& pose, bool fixed)
{
  //ROS_INFO("check pose:t:%lf",pose.t());
  VertexPose* pose_vertex = new VertexPose(pose, fixed);
  pose_vec_.push_back( pose_vertex );
  //ROS_INFO("check t:%lf",pose_vec_.back()->t());
  return;
}

void TimedElasticBand::addPose(const Eigen::Ref<const Eigen::Vector2d>& position,double t, double theta, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(position,t , theta, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}

 void TimedElasticBand::addPose(double x, double y, double t, double theta, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(x, y, t, theta, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}

void TimedElasticBand::deletePose(int index)
{
  ROS_ASSERT(index<pose_vec_.size());
  delete pose_vec_.at(index);
  pose_vec_.erase(pose_vec_.begin()+index);
}

void TimedElasticBand::deletePoses(int index, int number)
{
  ROS_ASSERT(index+number<=(int)pose_vec_.size());
  for (int i = index; i<index+number; ++i)
    delete pose_vec_.at(i);
  pose_vec_.erase(pose_vec_.begin()+index, pose_vec_.begin()+index+number);
}


void TimedElasticBand::insertPose(int index, const PoseSE3& pose)
{
  VertexPose* pose_vertex = new VertexPose(pose);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TimedElasticBand::insertPose(int index, const Eigen::Ref<const Eigen::Vector2d>& position,double t, double theta)
{
  VertexPose* pose_vertex = new VertexPose(position, t, theta);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TimedElasticBand::insertPose(int index, double x, double y,double t, double theta)
{
  VertexPose* pose_vertex = new VertexPose(x, y, t, theta);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}


void TimedElasticBand::clearTimedElasticBand()
{
  for (PoseSequence::iterator pose_it = pose_vec_.begin(); pose_it != pose_vec_.end(); ++pose_it)
    delete *pose_it;
  pose_vec_.clear();
}


void TimedElasticBand::setPoseVertexFixed(int index, bool status)
{
  ROS_ASSERT(index<sizePoses());
  pose_vec_.at(index)->setFixed(status);   
}
void TimedElasticBand::autoResize(double dt_ref, double dt_hysteresis, int min_samples, int max_samples, bool fast_mode,double max_vel_x,double max_vel_theta)
{  

  /// iterate through all TEB states and add/remove states!
  bool modified = true;
   //int n = sizePoses();
  for (int rep = 0; rep < 100 && modified; ++rep) // actually it should be while(), but we want to make sure to not get stuck in some oscillation, hence max 100 repitions.
  {
    modified = false;

    for(int i=1; i < sizePoses(); ++i) // TimeDiff connects Point(i) with Point(i+1)
    {
        double timeDiff = Pose(i).t()-Pose(i-1).t();
        //ROS_INFO("timediff:%lf",timeDiff);
      if(timeDiff > dt_ref + dt_hysteresis && sizePoses()<max_samples)
      {
        PoseSE3 newPose = PoseSE3::average(Pose(i-1),Pose(i));
        double dt = estimateDeltaT(Pose(i-1), newPose.toPoseSE2(), max_vel_x, max_vel_theta);
        newPose.t() = Pose(i-1).t()+dt;
        dt = estimateDeltaT(newPose,Pose(i).toPoseSE2(),  max_vel_x, max_vel_theta);
        Pose(i).t() = newPose.t()+dt;
        //ROS_INFO("insert %d",i);
        insertPose(i, newPose );
        modified = true;
      }
      else if(timeDiff < dt_ref - dt_hysteresis && sizePoses()>min_samples) // only remove samples if size is larger than min_samples.
      {
          //ROS_INFO("delete %d",i);
          double new_dt=0.0;
         if (i==sizePoses()-1)
         {
             timeDiffVertex()->dt() =estimateDeltaT(Pose(i-1), PoseGoal()->pose(), max_vel_x, max_vel_theta);
         }else
         {
             new_dt = estimateDeltaT(Pose(i-1), Pose(i+1).toPoseSE2(), max_vel_x, max_vel_theta);
             Pose(i+1).t() = Pose(i-1).t()+new_dt;
         }
         deletePose(i);


         modified = true;
      }
    }
    double timeDiff = timeDiffVertex()->dt();
    //ROS_INFO("timediff last:%lf",timeDiff);
    if(timeDiff > dt_ref + dt_hysteresis && sizePoses()<max_samples)
    {
        PoseSE2 newPose = PoseSE2::average(Pose(sizePoses()-1).toPoseSE2(),PoseGoal()->pose());
        PoseSE3 insertp = PoseSE3(newPose,BackPose().t()+timeDiff*0.5);
        //ROS_INFO("insert pose");
        //ROS_INFO("insert %d",sizePoses());
        double dt = estimateDeltaT(BackPose(), insertp.toPoseSE2(), max_vel_x, max_vel_theta);
        insertp.t() = BackPose().t()+dt;
        timeDiffVertex()->dt() = estimateDeltaT(insertp, PoseGoal()->pose(), max_vel_x, max_vel_theta);
        addPose(insertp);
        //ROS_INFO("end insert pose");
        //timeDiffVertex()->dt() = timeDiffVertex()->dt()*0.5;
        //ROS_INFO("end insert pose 1");
        modified = true;
    }
    else if(timeDiff < dt_ref - dt_hysteresis && sizePoses()>min_samples) // only remove samples if size is larger than min_samples.
    {
        //ROS_INFO("delete pose");
        if (sizePoses()>1)
        {
            //ROS_INFO("delete %d",sizePoses()-1);
            double dt = Pose(sizePoses()-1).t()-Pose(sizePoses()-2).t();
            deletePose(sizePoses()-1);
            //ROS_INFO("end delete pose");
            timeDiffVertex()->dt() += dt;
            modified = true;
        }

    }
    if (fast_mode) break;
  }
}


double TimedElasticBand::getSumOfAllTimeDiffs() const
{
    ROS_ASSERT(sizePoses()>=1);
  return Pose(sizePoses()-1).t()-Pose(0).t()+time_diff_goal_->dt();
}

double TimedElasticBand::getSumOfTimeDiffsUpToIdx(int index) const
{
  ROS_ASSERT(index<sizePoses());
  return Pose(index).t() - Pose(0).t();
}

double TimedElasticBand::getAccumulatedDistance() const
{
  double dist = 0;

  for(int i=1; i<sizePoses(); ++i)
  {
      dist += (Pose(i).position() - Pose(i-1).position()).norm();
  }
  return dist;
}
double TimedElasticBand::getLength() const
{
    return getLengthUpToIdx(sizePoses()-1);
}
double TimedElasticBand::getLengthUpToIdx(int index) const
{
    double dist = 0;
    for (int i=1;i<=index;++i)
    {
        double space_dist_2 = (Pose(i).position() - Pose(i-1).position()).squaredNorm();
        double time_dist = Pose(i).t() - Pose(i-1).t();
        dist += sqrt(space_dist_2 + pow(time_scale_*time_dist,2));
    }
    return dist;
}

bool TimedElasticBand::initTrajectoryToGoal(const PoseSE2& start, const PoseSE2& goal, double diststep, double max_vel_x, int min_samples, bool guess_backwards_motion)
{
  if (!isInit())
  {
    //ROS_WARN("into initTrajectoryToGoal");
      double now = ros::Time::now().toSec();
    PoseSE3 startSE3(start.position(),now ,start.theta());
    addPose(startSE3); // add starting point
    setPoseVertexFixed(0,true); // StartConf is a fixed constraint during optimization

    double timestep = 0.1;
        
    if (diststep!=0)
    {
      Eigen::Vector2d point_to_goal;
      point_to_goal = goal.position()-start.position();
      double dir_to_goal = std::atan2(point_to_goal[1],point_to_goal[0]); // direction to goal
      double dx = diststep*std::cos(dir_to_goal);
      double dy = diststep*std::sin(dir_to_goal);
      double orient_init;
      //ROS_INFO("DEBUG:ininer local_pose");
      orient_init = dir_to_goal;


      // check if the goal is behind the start pose (w.r.t. start orientation)
      if (guess_backwards_motion && point_to_goal.dot(start.orientationUnitVec()) < 0)
        orient_init = g2o::normalize_theta(orient_init+M_PI);
      // TODO: timestep ~ max_vel_x_backwards for backwards motions
      
      double dist_to_goal = point_to_goal.norm();
      double no_steps_d = dist_to_goal/std::abs(diststep); // ignore negative values
      unsigned int no_steps = (unsigned int) std::floor(no_steps_d);

      if (max_vel_x > 0) timestep = diststep / max_vel_x;
      
      for (unsigned int i=1; i<=no_steps; i++) // start with 1! starting point had index 0
      {
        if (i==no_steps && no_steps_d==(float) no_steps) 
            break; // if last conf (depending on stepsize) is equal to goal conf -> leave loop
        addPose(start.x()+i*dx,start.y()+i*dy,now + timestep*i,orient_init);
      }

    }
    
    // if number of samples is not larger than min_samples, insert manually
    double dt = 0.1;
    if (max_vel_x>0)
      dt = (goal.position()-BackPose().position()).norm() / max_vel_x;
    PoseSE3 goalSE3(goal.position(),BackPose().t()+dt,goal.theta());
    if ( sizePoses() < min_samples-1 )
    {
      ROS_DEBUG("initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
      //ROS_WARN("DEBUG:size pose:%d",sizePoses());
      while (sizePoses() < min_samples-1) // subtract goal point that will be added later
      {
        // simple strategy: interpolate between the current pose and the goal
        PoseSE3 intermediate_pose = PoseSE3::average(BackPose(), goalSE3);
        double dt = (intermediate_pose.position() - BackPose().position()).norm()/ max_vel_x;
        intermediate_pose.t() = BackPose().t()+dt;
        addPose( intermediate_pose); // let the optimier correct the timestep (TODO: better initialization
      }
    }
    time_diff_goal_ = new VertexTimeDiff(goalSE3.t()-BackPose().t());
    pose_goal_ = new VertexPose2D(goalSE3.toPoseSE2());
    // add goal
    pose_goal_->setFixed(true);
    //addPose(goalSE3); // add goal point
    //setPoseVertexFixed(sizePoses()-1,true);
  }
  else // size!=0
  {
    ROS_WARN("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
    ROS_WARN("Number of TEB configurations: %d",(unsigned int) sizePoses());
    return false;
  }
  return true;
}


bool TimedElasticBand::initTrajectoryToGoal(const std::vector<geometry_msgs::PoseStamped>& plan, double max_vel_x,
                                            double max_vel_theta, bool estimate_orient, int min_samples,
                                            bool guess_backwards_motion)
{
  if (!isInit())
  {
      double now = ros::Time::now().toSec();
      PoseSE3 startSE3(plan.front().pose,now);
      PoseSE2 goal(plan.back().pose);
    //ROS_INFO("begin teb init");
    addPose(startSE3); // add starting point with given orientation
    setPoseVertexFixed(0,true); // StartConf is a fixed constraint during optimization

    bool backwards = false;
    if (guess_backwards_motion && (goal.position()-startSE3.position()).dot(startSE3.orientationUnitVec()) < 0) // check if the goal is behind the start pose (w.r.t. start orientation)
        backwards = true;
    // TODO: dt ~ max_vel_x_backwards for backwards motions
    //ROS_INFO("DEBUG:ininer local_pose2");
    //ROS_INFO("now time:%lf",now);
    for (int i=1; i<(int)plan.size()-1; ++i)
    {
        double yaw;
        if (estimate_orient)
        {
            // get yaw from the orientation of the distance vector between pose_{i+1} and pose_{i}
            double dx = plan[i+1].pose.position.x - plan[i].pose.position.x;
            double dy = plan[i+1].pose.position.y - plan[i].pose.position.y;
            yaw = g2o::normalize_theta(std::atan2(dy,dx));
            if (backwards)
                yaw = g2o::normalize_theta(yaw+M_PI);
        }
        else 
        {
            yaw = g2o::normalize_theta(tf::getYaw(plan[i].pose.orientation));
        }
        PoseSE2 intermediate_pose(plan[i].pose.position.x, plan[i].pose.position.y, yaw);
        double dt = estimateDeltaT(BackPose(), intermediate_pose, max_vel_x, max_vel_theta);
        //ROS_INFO("dt:%lf",dt);
        PoseSE3 intermediate_pose_SE3(plan[i].pose.position.x, plan[i].pose.position.y,BackPose().t()+dt, yaw);
         //ROS_INFO("posese3: t:%lf",intermediate_pose_SE3.t());
        addPose(intermediate_pose_SE3);
    }
    //ROS_INFO("mid teb init");
    //for (int i=0;i<sizePoses();++i)
    //{
        //ROS_INFO("begin init index:%d x:%lf y:%lf t:%lf theta:%lf",i,Pose(i).x(),Pose(i).y(),Pose(i).t(),Pose(i).theta());
    //}
    double dt = estimateDeltaT(BackPose(), goal, max_vel_x, max_vel_theta);
    PoseSE3 goalSE3(goal.position(),BackPose().t()+dt,goal.theta());
    // if number of samples is not larger than min_samples, insert manually
    if ( sizePoses() < min_samples-1 )
    {
      ROS_DEBUG("initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
      while (sizePoses() < min_samples-1) // subtract goal point that will be added later
      {
        // simple strategy: interpolate between the current pose and the goal
          PoseSE3 intermediate_pose = PoseSE3::average(BackPose(), goalSE3);
          double dt = (intermediate_pose.position() - BackPose().position()).norm()/ max_vel_x;
          intermediate_pose.t() = BackPose().t()+dt;
          addPose( intermediate_pose);  // let the optimier correct the timestep (TODO: better initialization
      }
    }
    //ROS_INFO("add goal");
    // Now add final state with given orientation
    time_diff_goal_ = new VertexTimeDiff(goalSE3.t()-BackPose().t());
    pose_goal_ = new VertexPose2D(goalSE3.toPoseSE2());
    // add goal
    pose_goal_->setFixed(true);

    //addPose(goalSE3);
    //setPoseVertexFixed(sizePoses()-1,true);
    //ROS_INFO("end teb init");
    //setPoseVertexFixed(sizePoses()-1,true); // GoalConf is a fixed constraint during optimization
  }
  else // size!=0
  {
    ROS_WARN("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
    ROS_WARN("Number of TEB configurations: %d", sizePoses());
    return false;
  }
  
  return true;
}


int TimedElasticBand::findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_point, double* distance, int begin_idx) const
{
  int n = sizePoses();
  if (begin_idx < 0 || begin_idx >= n)
    return -1;

  double min_dist_sq = std::numeric_limits<double>::max();
  int min_idx = -1;
  
  for (int i = begin_idx; i < n; i++)
  {
    double dist_sq = (ref_point - Pose(i).position()).squaredNorm();
    if (dist_sq < min_dist_sq)
    {
      min_dist_sq = dist_sq;
      min_idx = i;
    }
  }
  // double dist_sq = (ref_point - pose_goal_->pose().position()).squaredNorm();
  // if (dist_sq < min_dist_sq)
  // {
  //   min_dist_sq = dist_sq;
  //   min_idx = n;
  // }

  if (distance)
    *distance = std::sqrt(min_dist_sq);

  return min_idx;
}
int TimedElasticBand::findClosestTrajectoryPose3D(const Eigen::Ref<const Eigen::Vector3d> &ref_point, double *distance, int begin_idx) const
{
    int n = sizePoses();
    if (begin_idx < 0 || begin_idx >= n)
      return -1;

    double min_dist_sq = std::numeric_limits<double>::max();
    int min_idx = -1;
    //#pragma omp parallel for
    for (int i = begin_idx; i < n; i++)
    {
      double dist_sq = computeTimeSpaceDistance(ref_point,Pose(i));
      if (dist_sq < min_dist_sq)
      {
        min_dist_sq = dist_sq;
        min_idx = i;
      }
    }
    // double dist_sq = computeTimeSpaceDistance(ref_point,PoseSE3(pose_goal_->pose(),BackPose().t()+time_diff_goal_->dt()));
    // if (dist_sq < min_dist_sq)
    // {
    //   min_dist_sq = dist_sq;
    //   min_idx = n;
    // }
    if (distance)
      *distance = std::sqrt(min_dist_sq);

    return min_idx;
}
std::vector<int> TimedElasticBand::findTrajectoryPose3DInDist(const Eigen::Ref<const Eigen::Vector3d>& ref_point,double distance, double time_scale) const
{
  std::vector<int> ans;
  int n = sizePoses();
  for (int i = 1; i < n; i++)
  {
    double dist_sq = computeTimeSpaceDistance(ref_point,Pose(i));
    if (dist_sq < distance)
    {
      ans.push_back(i);
    }
  }
  return ans;
}
    
int TimedElasticBand::findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_line_start, const Eigen::Ref<const Eigen::Vector2d>& ref_line_end, double* distance) const
{
  double min_dist = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = 0; i < sizePoses(); i++)
  {
    Eigen::Vector2d point = Pose(i).position();
    double dist = distance_point_to_segment_2d(point, ref_line_start, ref_line_end);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_idx = i;
    }
  }
  // double dist = distance_point_to_segment_2d(pose_goal_->pose().position(), ref_line_start, ref_line_end);
  // if (dist < min_dist)
  // {
  //   min_dist = dist;
  //   min_idx = sizePoses();
  // }
  if (distance)
    *distance = min_dist;
  return min_idx;
}
int TimedElasticBand::findClosestTrajectoryPose3D(const Eigen::Ref<const Eigen::Vector3d> &ref_line_start, const Eigen::Ref<const Eigen::Vector3d> &ref_line_end, double *distance,double time_scale) const
{
    double min_dist = std::numeric_limits<double>::max();
    int min_idx = -1;
    Eigen::Vector3d scale(1.0,1.0,time_scale);
    //#pragma omp parallel for
    for (int i = 0; i < sizePoses(); i++)
    {
        //ROS_INFO("pose:%lf %lf %lf",Pose(i).position()[0],Pose(i).position()[1],Pose(i).t());
        //ROS_INFO("line1:%lf %lf %lf",ref_line_start[0],ref_line_start[1],ref_line_start[2]);
        //ROS_INFO("line1:%lf %lf %lf",ref_line_end[0],ref_line_end[1],ref_line_end[2]);
        Eigen::Vector3d pose(Pose(i).position()[0],Pose(i).position()[1],Pose(i).t());
      double dist = distance_point_to_segment_3d(pose.cwiseProduct(scale), ref_line_start.cwiseProduct(scale),
                                                 ref_line_end.cwiseProduct(scale),true);

      if (dist < min_dist&&dist>0)
      {
        min_dist = dist;
        min_idx = i;
      }
      //ROS_INFO("compute dist is:%lf min_dist is:%lf min_idx:%d",dist,min_dist,min_idx);
    }
    // double dist = distance_point_to_segment_3d(Eigen::Vector3d(pose_goal_->x(),pose_goal_->y(),BackPose().t()+time_diff_goal_->dt()), ref_line_start, ref_line_end);
    // if (dist < min_dist)
    // {
    //   min_dist = dist;
    //   min_idx = sizePoses();
    // }
    if (distance)
      *distance = min_dist;
    return min_idx;
}
std::vector<int> TimedElasticBand::findTrajectoryPose3DInDist(const Eigen::Ref<const Eigen::Vector3d>& ref_line_start, const Eigen::Ref<const Eigen::Vector3d>& ref_line_end,double distance, double time_scale) const
{
  std::vector<int> ans;
  Eigen::Vector3d scale(1.0,1.0,time_scale);
  for (int i = 1; i < sizePoses(); i++)
  {
    Eigen::Vector3d pose(Pose(i).position()[0],Pose(i).position()[1],Pose(i).t());
    double dist = distance_point_to_segment_3d(pose.cwiseProduct(scale), ref_line_start.cwiseProduct(scale),
                                                ref_line_end.cwiseProduct(scale),true);
    if (dist < distance&&dist>0)
    {
      ans.push_back(i);
    }
  }
  return ans;
}

int TimedElasticBand::findClosestTrajectoryPose(const Point2dContainer& vertices, double* distance) const
{
  if (vertices.empty())
    return 0;
  else if (vertices.size() == 1)
    return findClosestTrajectoryPose(vertices.front());
  else if (vertices.size() == 2)
    return findClosestTrajectoryPose(vertices.front(), vertices.back());
  
  double min_dist = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = 0; i < sizePoses(); i++)
  {
    Eigen::Vector2d point = Pose(i).position();
    double dist_to_polygon = std::numeric_limits<double>::max();
    for (int j = 0; j < (int) vertices.size()-1; ++j)
    {
      dist_to_polygon = std::min(dist_to_polygon, distance_point_to_segment_2d(point, vertices[j], vertices[j+1]));
    }
    dist_to_polygon = std::min(dist_to_polygon, distance_point_to_segment_2d(point, vertices.back(), vertices.front()));
    if (dist_to_polygon < min_dist)
    {
      min_dist = dist_to_polygon;
      min_idx = i;
    }
  }
  // double dist_to_polygon = std::numeric_limits<double>::max();
  // for (int j = 0; j < (int) vertices.size()-1; ++j)
  // {
  //   dist_to_polygon = std::min(dist_to_polygon, distance_point_to_segment_2d(pose_goal_->position(), vertices[j], vertices[j+1]));
  // }
  // dist_to_polygon = std::min(dist_to_polygon, distance_point_to_segment_2d(pose_goal_->position(), vertices.back(), vertices.front()));
  // if (dist_to_polygon < min_dist)
  // {
  //   min_dist = dist_to_polygon;
  //   min_idx = sizePoses();
  // }
  if (distance)
    *distance = min_dist;

  return min_idx;
}


int TimedElasticBand::findClosestTrajectoryPose(const Obstacle& obstacle, double* distance) const
{
  const PointObstacle* pobst = dynamic_cast<const PointObstacle*>(&obstacle);
  if (pobst)
    return findClosestTrajectoryPose(pobst->position(), distance);

  const LineObstacle* lobst = dynamic_cast<const LineObstacle*>(&obstacle);
  if (lobst)
    return findClosestTrajectoryPose(lobst->start(), lobst->end(), distance);
  
  const PolygonObstacle* polyobst = dynamic_cast<const PolygonObstacle*>(&obstacle);
  if (polyobst)
    return findClosestTrajectoryPose(polyobst->vertices(), distance);
  
  return findClosestTrajectoryPose(obstacle.getCentroid(), distance);  
}


void TimedElasticBand::updateAndPruneTEB(boost::optional<const PoseSE2&> new_start, boost::optional<const PoseSE2&> new_goal, int min_samples,double max_vel_x,double max_vel_theta)
{
  // first and simple approach: change only start confs (and virtual start conf for inital velocity)
  // TEST if optimizer can handle this "hard" placement

  if (new_start && sizePoses()>0)
  {    
    // find nearest state (using l2-norm) in order to prune the trajectory
    // (remove already passed states)
    double dist_cache = (new_start->position()- Pose(0).position()).norm();
    double dist;
    int lookahead = std::min<int>( sizePoses()-min_samples, 10); // satisfy min_samples, otherwise max 10 samples
    //ROS_INFO("look ahead:%d",lookahead);
    int nearest_idx = 0;
    for (int i = 1; i<=lookahead; ++i)
    {
      dist = (new_start->position()- Pose(i).position()).norm();
      if (dist<dist_cache)
      {
        dist_cache = dist;
        nearest_idx = i;
      }
      else break;
    }
    
    // prune trajectory at the beginning (and extrapolate sequences at the end if the horizon is fixed)
    if (nearest_idx>0)
    {
      // nearest_idx is equal to the number of samples to be removed (since it counts from 0 ;-) )
      // WARNING delete starting at pose 1, and overwrite the original pose(0) with new_start, since Pose(0) is fixed during optimization!
      deletePoses(1, nearest_idx);  // delete first states such that the closest state is the new first one
    }
    
    // update start

    //double dt = estimateDeltaT()max_vel_x>0?(Pose(1).position()-new_start->position()).norm()/max_vel_x:0.0;
    Pose(0).position() = new_start->position();
    Pose(0).theta() = new_start->theta();
    Pose(0).t() =ros::Time::now().toSec();
  }
  
  if (new_goal && sizePoses()>0)
  {
    pose_goal_->position() = new_goal->position();
    pose_goal_->theta() = new_goal->theta();
    //if (sizePoses()>1)
    //{
        //BackPose().t() = Pose(sizePoses()-2).t() + estimateDeltaT(Pose(sizePoses()-2),BackPose().toPoseSE2(), max_vel_x,max_vel_theta);
    //}
  }
};


bool TimedElasticBand::isTrajectoryInsideRegion(double radius, double max_dist_behind_robot, int skip_poses)
{
    if (sizePoses()<=0)
        return true;
    
    double radius_sq = radius*radius;
    double max_dist_behind_robot_sq = max_dist_behind_robot*max_dist_behind_robot;
    Eigen::Vector2d robot_orient = Pose(0).orientationUnitVec();

    for (int i=1; i<sizePoses(); i=i+skip_poses+1)
    {
        Eigen::Vector2d dist_vec = Pose(i).position()-Pose(0).position();
        double dist_sq = dist_vec.squaredNorm();
        
        if (dist_sq > radius_sq)
        {
            ROS_INFO("outside robot");
            return false;
        }
        
        // check behind the robot with a different distance, if specified (or >=0)
        if (max_dist_behind_robot >= 0 && dist_vec.dot(robot_orient) < 0 && dist_sq > max_dist_behind_robot_sq)
        {
            ROS_INFO("outside robot behind");
            return false;
        }
        
    }
    return true;
}
// modified by Hu Xingyu
void TimedElasticBand::updatePose(geometry_msgs::PoseStamped human_pose)
{
    Eigen::Vector2d human_location(human_pose.pose.position.x,human_pose.pose.position.y);
    for (int i=1;i<sizePoses();i++)
    {
        Eigen::Vector2d direction =  human_location - Pose(i).position();
        Pose(i).theta() = std::atan2(direction[1],direction[0]);
        //ROS_INFO("pose %d theta: %lf",i,Pose(i).theta());
    }
    Eigen::Vector2d direction =  human_location - pose_goal_->position();
    pose_goal_->theta() = std::atan2(direction[1],direction[0]);
}
double TimedElasticBand::computeTimeSpaceDistance(const Eigen::Vector3d &ref_point, const PoseSE3 &pose) const
{
    double space_dist_2 = pow(ref_point[0]-pose.x(),2) + pow(ref_point[1]-pose.y(),2);
    double t_dist_2 = pow(time_scale_*(ref_point[2]-pose.t()),2);
    return sqrt(t_dist_2+space_dist_2);
}

// end modified by Hu Xingyu



} // namespace teb_local_planner
