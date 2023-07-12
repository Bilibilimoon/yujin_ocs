/**
 * @file /include/yocs_velocity_smoother/velocity_smoother_nodelet.hpp
 *
 * @brief Velocity smoother interface
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef YUJIN_OCS_VELOCITY_SMOOTHER_HPP_
#define YUJIN_OCS_VELOCITY_SMOOTHER_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace yocs_velocity_smoother {

/*****************************************************************************
** VelocitySmoother
*****************************************************************************/

class VelocitySmoother
{
public:
  VelocitySmoother(const std::string &name);

  ~VelocitySmoother()
  {
    if (dynamic_reconfigure_server != NULL)
      delete dynamic_reconfigure_server;
  }

  bool init(ros::NodeHandle& nh);
  void spin();
  void shutdown() { shutdown_req = true; };
  std::mutex locker;

private:
  enum RobotFeedbackType
  {
    NONE,
    ODOMETRY,
    COMMANDS
  } robot_feedback;  /**< 用作机器人速度反馈的来源 */

  std::string name;
  bool quiet;        /**< 减少一些警告，这些警告是由于速度多路复用而不可避免的 **/
  double speed_lim_v, accel_lim_v, decel_lim_v;
  double speed_lim_w, accel_lim_w, decel_lim_w;
  double decel_factor;

  double frequency;

  geometry_msgs::Twist last_cmd_vel;
  geometry_msgs::Twist  current_vel;
  geometry_msgs::Twist   target_vel;

  bool                 shutdown_req; /**< 节点请求关闭；终止工作线程 */
  bool                 input_active;
  double                cb_avg_time;
  ros::Time            last_cb_time;
  std::vector<double> period_record; /**< 最新速度命令之间的历史周期 */
  unsigned int             pr_next; /**< 填充周期记录缓冲区的下一个位置 */

  ros::Subscriber odometry_sub;    /**< 来自里程计的当前速度 */
  ros::Subscriber current_vel_sub; /**< 来自发送给机器人的命令的当前速度，不一定是由此节点发送的  */
  ros::Subscriber raw_in_vel_sub;  /**< 输入的原始速度命令 */
  ros::Publisher  smooth_vel_pub;  /**< 输出平滑的速度命令 */

  void velocityCB(const geometry_msgs::Twist::ConstPtr& msg);
  void robotVelCB(const geometry_msgs::Twist::ConstPtr& msg);
  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; };

  double median(std::vector<double> values) {
    // 返回双精度浮点数向量的中位数元素
    nth_element(values.begin(), values.begin() + values.size()/2, values.end());
    return values[values.size()/2];
  };

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<yocs_velocity_smoother::paramsConfig> *             dynamic_reconfigure_server;
  dynamic_reconfigure::Server<yocs_velocity_smoother::paramsConfig>::CallbackType dynamic_reconfigure_callback;
  void reconfigCB(yocs_velocity_smoother::paramsConfig &config, uint32_t unused_level);
  
};

} // yocs_namespace velocity_smoother

#endif /* YUJIN_OCS_VELOCITY_SMOOTHER_HPP_ */
