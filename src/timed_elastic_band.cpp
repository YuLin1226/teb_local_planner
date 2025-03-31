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

#include <teb_local_planner/timed_elastic_band.h>

#include <limits>

namespace teb_local_planner
{

namespace
{
  /**
   * estimate the time to move from start to end.
   * Assumes constant velocity for the motion.
   */
  double estimateDeltaT(const PoseSE2& start, const PoseSE2& end,
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


TimedElasticBand::TimedElasticBand()
{
}

TimedElasticBand::~TimedElasticBand()
{
  ROS_DEBUG("Destructor Timed_Elastic_Band...");
  clearTimedElasticBand();
}

// 添加優化的Vertex Pose
void TimedElasticBand::addPose(const PoseSE2& pose, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(pose, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}

// 添加優化的Vertex Pose
void TimedElasticBand::addPose(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(position, theta, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}

// 添加優化的Vertex Pose
 void TimedElasticBand::addPose(double x, double y, double theta, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(x, y, theta, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}

// 添加優化的Vertex Time Difference
void TimedElasticBand::addTimeDiff(double dt, bool fixed)
{
  ROS_ASSERT_MSG(dt > 0., "Adding a timediff requires a positive dt");
  VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt, fixed);
  timediff_vec_.push_back( timediff_vertex );
  return;
}


void TimedElasticBand::addPoseAndTimeDiff(double x, double y, double angle, double dt)
{
  if (sizePoses() != sizeTimeDiffs())
  {
    addPose(x,y,angle,false);
    addTimeDiff(dt,false);
  }
  else
    ROS_ERROR("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  return;
}



void TimedElasticBand::addPoseAndTimeDiff(const PoseSE2& pose, double dt)
{
  if (sizePoses() != sizeTimeDiffs())
  {
    addPose(pose,false);
    addTimeDiff(dt,false);
  } else
    ROS_ERROR("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  return;
}

void TimedElasticBand::addPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, double dt)
{
  if (sizePoses() != sizeTimeDiffs())
  {
    addPose(position, theta,false);
    addTimeDiff(dt,false);
  } else
    ROS_ERROR("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
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

void TimedElasticBand::deleteTimeDiff(int index)
{
  ROS_ASSERT(index<(int)timediff_vec_.size());
  delete timediff_vec_.at(index);
  timediff_vec_.erase(timediff_vec_.begin()+index);
}

void TimedElasticBand::deleteTimeDiffs(int index, int number)
{
  ROS_ASSERT(index+number<=timediff_vec_.size());
  for (int i = index; i<index+number; ++i)
    delete timediff_vec_.at(i);
  timediff_vec_.erase(timediff_vec_.begin()+index, timediff_vec_.begin()+index+number);
}

void TimedElasticBand::insertPose(int index, const PoseSE2& pose)
{
  VertexPose* pose_vertex = new VertexPose(pose);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TimedElasticBand::insertPose(int index, const Eigen::Ref<const Eigen::Vector2d>& position, double theta)
{
  VertexPose* pose_vertex = new VertexPose(position, theta);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TimedElasticBand::insertPose(int index, double x, double y, double theta)
{
  VertexPose* pose_vertex = new VertexPose(x, y, theta);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TimedElasticBand::insertTimeDiff(int index, double dt)
{
  VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt);
  timediff_vec_.insert(timediff_vec_.begin()+index, timediff_vertex);
}


void TimedElasticBand::clearTimedElasticBand()
{
  for (PoseSequence::iterator pose_it = pose_vec_.begin(); pose_it != pose_vec_.end(); ++pose_it)
    delete *pose_it;
  pose_vec_.clear();

  for (TimeDiffSequence::iterator dt_it = timediff_vec_.begin(); dt_it != timediff_vec_.end(); ++dt_it)
    delete *dt_it;
  timediff_vec_.clear();
}


void TimedElasticBand::setPoseVertexFixed(int index, bool status)
{
  ROS_ASSERT(index<sizePoses());
  pose_vec_.at(index)->setFixed(status);
}

void TimedElasticBand::setTimeDiffVertexFixed(int index, bool status)
{
  ROS_ASSERT(index<sizeTimeDiffs());
  timediff_vec_.at(index)->setFixed(status);
}


void TimedElasticBand::autoResize(double dt_ref, double dt_hysteresis, int min_samples, int max_samples, bool fast_mode)
{
  ROS_ASSERT(sizeTimeDiffs() == 0 || sizeTimeDiffs() + 1 == sizePoses());
  // 遍历所有的TEB状态(pose+time),增加和删减状态
  bool modified = true;

  for (int rep = 0; rep < 100 && modified; ++rep) // actually it should be while(), but we want to make sure to not get stuck in some oscillation, hence max 100 repitions.
  {
    modified = false;

    for(int i=0; i < sizeTimeDiffs(); ++i) // TimeDiff connects Point(i) with Point(i+1)
    {
      if(TimeDiff(i) > dt_ref + dt_hysteresis && sizeTimeDiffs()<max_samples)
      {
          // Force the planner to have equal timediffs between poses (dt_ref +/- dt_hyteresis).
          // (new behaviour)
          if (TimeDiff(i) > 2*dt_ref)
          {
              double newtime = 0.5*TimeDiff(i);

              TimeDiff(i) = newtime;
              insertPose(i+1, PoseSE2::average(Pose(i),Pose(i+1)) );
              insertTimeDiff(i+1,newtime);

              i--; // check the updated pose diff again
              modified = true;
          }
          else
          {
              if (i < sizeTimeDiffs() - 1)
              {
                  timediffs().at(i+1)->dt()+= timediffs().at(i)->dt() - dt_ref;
              }
              timediffs().at(i)->dt() = dt_ref;
          }
      }
      else if(TimeDiff(i) < dt_ref - dt_hysteresis && sizeTimeDiffs()>min_samples) // only remove samples if size is larger than min_samples.
      {
        //ROS_DEBUG("teb_local_planner: autoResize() deleting bandpoint i=%u, #TimeDiffs=%lu",i,sizeTimeDiffs());

        if(i < ((int)sizeTimeDiffs()-1))
        {
          TimeDiff(i+1) = TimeDiff(i+1) + TimeDiff(i);
          deleteTimeDiff(i);
          deletePose(i+1);
          i--; // check the updated pose diff again
        }
        else
        { // last motion should be adjusted, shift time to the interval before
          TimeDiff(i-1) += TimeDiff(i);
          deleteTimeDiff(i);
          deletePose(i);
        }

        modified = true;
      }
    }
    if (fast_mode) break;
  }
}


double TimedElasticBand::getSumOfAllTimeDiffs() const
{
  double time = 0;

  for(TimeDiffSequence::const_iterator dt_it = timediff_vec_.begin(); dt_it != timediff_vec_.end(); ++dt_it)
  {
      time += (*dt_it)->dt();
  }
  return time;
}

double TimedElasticBand::getSumOfTimeDiffsUpToIdx(int index) const
{
  ROS_ASSERT(index<=timediff_vec_.size());

  double time = 0;

  for(int i = 0; i < index; ++i)
  {
    time += timediff_vec_.at(i)->dt();
  }

  return time;
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

// 在起点和终点之间初始化出一条轨迹
// 用给定的采样距离对连接起点和终点的直线进行插值，采样距离可以是diststep（欧式空间）
// 位姿之间的时间差初始化为timestep.
// 如果diststep是零，产生的轨迹只包含起始点和终点。
bool TimedElasticBand::initTrajectoryToGoal(const PoseSE2& start, const PoseSE2& goal, double diststep, double max_vel_x, int min_samples, bool guess_backwards_motion)
{
  if (!isInit())
  {
    addPose(start); // 添加初始点
    setPoseVertexFixed(0,true); // 优化过程中起始点是固定的约束

    double timestep = 0.1;

    if (diststep!=0) // 起点和终点之间进行插值
    {
      Eigen::Vector2d point_to_goal = goal.position()-start.position();
      double dir_to_goal = std::atan2(point_to_goal[1],point_to_goal[0]); // 目标点方向
      double dx = diststep*std::cos(dir_to_goal);
      double dy = diststep*std::sin(dir_to_goal);
      double orient_init = dir_to_goal;
      // 检查目标点是否在起始位姿的后方 (基于起始点的朝向)
      if (guess_backwards_motion && point_to_goal.dot(start.orientationUnitVec()) < 0)
        orient_init = g2o::normalize_theta(orient_init+M_PI);

      double dist_to_goal = point_to_goal.norm();
      double no_steps_d = dist_to_goal/std::abs(diststep); // 忽略正负号
      unsigned int no_steps = (unsigned int) std::floor(no_steps_d); // 取整

      if (max_vel_x > 0) timestep = diststep / max_vel_x;

      for (unsigned int i=1; i<=no_steps; i++) // 从i=1开始
      {
        if (i==no_steps && no_steps_d==(float) no_steps)
            break; // 如果最后一个插值和目标点一样则退出循环
        addPoseAndTimeDiff(start.x()+i*dx,start.y()+i*dy,orient_init,timestep);
      }

    }

    // 如果样本数比min_samples-1少，手动插值
    if ( sizePoses() < min_samples-1 )
    {
      ROS_DEBUG("initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
      while (sizePoses() < min_samples-1) // 没有考虑目标点，之后会添加进来
      {
        // simple strategy: interpolate between the current pose and the goal
        // ？？？
        PoseSE2 intermediate_pose = PoseSE2::average(BackPose(), goal);
        if (max_vel_x > 0) timestep = (intermediate_pose.position()-BackPose().position()).norm()/max_vel_x;
        addPoseAndTimeDiff( intermediate_pose, timestep ); // let the optimier correct the timestep (TODO: better initialization
      }
    }

    // 添加目标点
    if (max_vel_x > 0) timestep = (goal.position()-BackPose().position()).norm()/max_vel_x;
    addPoseAndTimeDiff(goal,timestep);
    setPoseVertexFixed(sizePoses()-1,true); // 优化过程中目标点的约束是固定的
  }
  else // 位姿数不为零，已经初始化过了
  {
    ROS_WARN("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
    ROS_WARN("Number of TEB configurations: %d, Number of TEB timediffs: %d",(unsigned int) sizePoses(),(unsigned int) sizeTimeDiffs());
    return false;
  }
  return true;
}


bool TimedElasticBand::initTrajectoryToGoal(const std::vector<geometry_msgs::PoseStamped>& plan, double max_vel_x, double max_vel_theta, bool estimate_orient, int min_samples, bool guess_backwards_motion)
{

  if (!isInit())
  {
    PoseSE2 start(plan.front().pose);
    PoseSE2 goal(plan.back().pose);

    addPose(start); // 添加起始点
    setPoseVertexFixed(0,true); // 优化过程中起始点是固定的约束

    bool backwards = false;
    if (guess_backwards_motion && (goal.position()-start.position()).dot(start.orientationUnitVec()) < 0) // 目标点是否在后面
        backwards = true;

    for (int i=1; i<(int)plan.size()-1; ++i)
    {
        double yaw;
        if (estimate_orient)
        {
            // 从pose(i)到pose(i+1)的向量得到yaw朝向
            double dx = plan[i+1].pose.position.x - plan[i].pose.position.x;
            double dy = plan[i+1].pose.position.y - plan[i].pose.position.y;
            yaw = std::atan2(dy,dx);
            if (backwards)
                yaw = g2o::normalize_theta(yaw+M_PI);
        }
        else
        {
            yaw = tf::getYaw(plan[i].pose.orientation);
        }
        PoseSE2 intermediate_pose(plan[i].pose.position.x, plan[i].pose.position.y, yaw);
        double dt = estimateDeltaT(BackPose(), intermediate_pose, max_vel_x, max_vel_theta);
        addPoseAndTimeDiff(intermediate_pose, dt);
    }

    // 如果样本数比min_samples-1少，手动插值
    if ( sizePoses() < min_samples-1 )
    {
      ROS_DEBUG("initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
      while (sizePoses() < min_samples-1) // 没有考虑目标点，之后会添加进来
      {
        // simple strategy: interpolate between the current pose and the goal
        PoseSE2 intermediate_pose = PoseSE2::average(BackPose(), goal);
        double dt = estimateDeltaT(BackPose(), intermediate_pose, max_vel_x, max_vel_theta);
        addPoseAndTimeDiff( intermediate_pose, dt ); // let the optimier correct the timestep (TODO: better initialization
      }
    }

    // 添加最后一个点
    double dt = estimateDeltaT(BackPose(), goal, max_vel_x, max_vel_theta);
    addPoseAndTimeDiff(goal, dt);
    setPoseVertexFixed(sizePoses()-1,true); // 优化过程中目标点的约束是固定的
  }
  else // 位姿数不为零，已经初始化过了
  {
    ROS_WARN("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
    ROS_WARN("Number of TEB configurations: %d, Number of TEB timediffs: %d", sizePoses(), sizeTimeDiffs());
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

  if (distance)
    *distance = std::sqrt(min_dist_sq);

  return min_idx;
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

  if (distance)
    *distance = min_dist;
  return min_idx;
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

/**
 * @brief  根据参考分辨率，通过添加或减少(pose,dt)改变轨迹尺寸.
 *
 * 改变轨迹尺寸以下场景有帮助:
 *
 * 	- 障碍物会让teb伸长，为了满足路径分辨率的要求和避免的太大或者太小步长导致的不当行为
 * 	  After clearance of obstacles, the teb should (re-) contract to its (time-)optimal version.
 *    - If the distance to the goal state is getting smaller,
 *      dt is decreasing as well. This leads to a heavily
 *      fine-grained discretization in combination with many
 *      discrete poses. Thus, the computation time will
 *      be/remain high and in addition numerical instabilities
 *      can appear (e.g. due to the division by a small \f$ \Delta T_i \f$).
 *
 * The implemented strategy checks all timediffs \f$ \Delta T_i \f$ and
 *
 * 	- inserts a new sample if \f$ \Delta T_i > \Delta T_{ref} + \Delta T_{hyst} \f$
 *    - removes a sample if \f$ \Delta T_i < \Delta T_{ref} - \Delta T_{hyst} \f$
 *
 * Each call only one new sample (pose-dt-pair) is inserted or removed.
 * @param dt_ref 参考分辨率
 * @param dt_hysteresis 避免震荡的迟滞缓冲
 * @param min_samples 轨迹改变尺寸后的最少样本数
 * @param max_samples 轨迹改变尺寸后的最大样本数
 * @param fast_mode 如果 true, 轨迹只迭代依次增加或者减少样本，false的话，不停的迭代直到没有位姿增加或者减少
 */
void TimedElasticBand::updateAndPruneTEB(boost::optional<const PoseSE2&> new_start, boost::optional<const PoseSE2&> new_goal, int min_samples)
{
  // first and simple approach: change only start confs (and virtual start conf for inital velocity)
  // TEST if optimizer can handle this "hard" placement

  if (new_start && sizePoses()>0)
  {
    // 找到最近的位姿（用l2-norm）,为了裁剪轨迹（去掉已经走过的点）
    double dist_cache = (new_start->position()- Pose(0).position()).norm();
    double dist;
    int lookahead = std::min<int>( sizePoses()-min_samples, 10); // lookahead不能超过10

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

    // 在开始出裁剪轨迹，如果要求horizon不变，就在尾部补充
    if (nearest_idx>0)
    {
      // nearest_idx is equal to the number of samples to be removed (since it counts from 0 ;-) )
      // WARNING delete starting at pose 1, and overwrite the original pose(0) with new_start, since Pose(0) is fixed during optimization!
      deletePoses(1, nearest_idx);  // delete first states such that the closest state is the new first one
      deleteTimeDiffs(1, nearest_idx); // delete corresponding time differences
    }

    // 更新起始点
    Pose(0) = *new_start;
  }

  if (new_goal && sizePoses()>0)
  {
    BackPose() = *new_goal;
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




} // namespace teb_local_planner
