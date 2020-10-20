#ifndef TF_RE_BROADCAST_H
#define TF_RE_BROADCAST_H
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <tf/transform_datatypes.h>

#include <thread>

class tf_re_broadcast
{
public:
  tf_re_broadcast(ros::NodeHandle pnh);


private:
  ros::NodeHandle* _pnh;
  ros::Rate* _loop;
  ros::Subscriber _cam_marker_sub,_robot_marker_sub;
  tf::TransformListener _listen;
  tf::TransformBroadcaster _broadcast;

  struct _names
  {
    tf::TransformBroadcaster _broadcast;
    tf::Transform _broadcast_tf;
    std::string _new_parent,_new_child;
    bool _inverse,_marker;
    std::string _marker_sub_topic;
  };

  _names _cam,_robot;
  double _pub_rate,_ar_marker_number,_robot_cam_height;
  bool _robot_cam_height_override;
  tf::Transform _broadcasted;


  void robotMarkerCB(const ar_track_alvar_msgs::AlvarMarkersConstPtr &mark);
  void camMarkerCB(const ar_track_alvar_msgs::AlvarMarkersConstPtr &mark);
  bool setup();
};

#endif // TF_RE_BROADCAST_H
