#ifndef FILTER_TF_CLASS_H_
#define FILTER_TF_CLASS_H_
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <ros/rate.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


class filter_tf_class
{
public:
  filter_tf_class(ros::NodeHandle pnh,bool launch);

private:
  bool setup();
  void setupnolaunch();
  bool checkTF();
  void filter();
  void filter_mv();
  bool checkOrigin(tf::Vector3 input,tf::Vector3 input2);
  bool checkRotation(tf::Quaternion input,tf::Quaternion input2);
  void run();

  ros::NodeHandle* _pnh;
  ros::Rate* _loop;
  bool _alpha_run,_launch,_quat_change;
  double _pub_rate,_quat_threshold,_vec_threshold,_alpha;
  std::string _parent,_child,_new_parent,_new_child;

  tf::Transform _buffertf;
  tf::TransformListener _listen;
  tf::TransformBroadcaster _broadcast;
  tf::StampedTransform _latest;
};

#endif

