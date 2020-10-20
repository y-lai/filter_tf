#include "filter_tf/filter_tf_class.h"


filter_tf_class::filter_tf_class(ros::NodeHandle pnh,bool launch):_alpha_run(true),_launch(launch)
{
  _pnh = new ros::NodeHandle(pnh);
  if(_launch && !setup())
  {
    ROS_INFO("Error when grabbing parameters. Please check roslaunch file.");
    return;
  }

  while(!checkTF())
  {
    ROS_INFO("Unable to find transform stated. Retrying in 3 seconds.");
    ros::Duration(3.0).sleep();
  }

  {
    _listen.lookupTransform(_parent,_child,ros::Time(0),_latest);
    _buffertf.setRotation(_latest.getRotation());
  }


  ROS_INFO("Running transform filter");
  run();
}

bool filter_tf_class::setup()
{
  bool temp = true;
  if(!_pnh->getParam("parent_frame",_parent))
  {
    ROS_INFO("Parent frame argument not found!");
    temp = false;
  }
  if(!_pnh->getParam("child_frame",_child))
  {
    ROS_INFO("Child frame argument not found!");
    temp = false;
  }
  if(!_pnh->getParam("new_parent_frame",_new_parent))
  {
    ROS_INFO("New parent frame argument not found!");
    temp = false;
  }
  if(!_pnh->getParam("new_child_frame",_new_child))
  {
    ROS_INFO("New child frame argument not found!");
    temp = false;
  }
  if(!_pnh->getParam("tf_publish_rate",_pub_rate))
  {
    ROS_INFO("Tf publish rate argument not found!");
    temp = false;
  }
  if(!_pnh->getParam("vector_threshold",_vec_threshold))
  {
    ROS_INFO("Vector threshold argument not found!");
    temp = false;
  }
  if(!_pnh->getParam("quaternion_threshold",_quat_threshold))
  {
    ROS_INFO("Quaternion threshold argument not found!");
    temp = false;
  }
  if(!_pnh->getParam("alpha",_alpha))
  {
    ROS_INFO("Alpha threshold argument not found! Using hard threshold.");
    _alpha_run = false;
  }
  if(!_pnh->getParam("quat_change",_quat_change))
  {
    ROS_INFO("Quat change argument not found! Default to false.");
    _quat_change = false;
  }
  return temp;
}

void filter_tf_class::setupnolaunch()
{
  _parent = "/camera_link";
  _child = "/ar_marker_13";
  _new_parent = "/camera_link";
  _new_child = "/marker";
  _pub_rate = 20;
  _vec_threshold = 0.0005;
  _quat_threshold = 0.0005;
  _alpha = 0.9;
  _alpha_run = true;
}

bool filter_tf_class::checkTF()
{
  try
  {
    _listen.waitForTransform(_parent,_child,ros::Time(0),ros::Duration(5.0));
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    return false;
  }
  return true;
}

void filter_tf_class::filter()  //filters, and then sets up _newtf as stampedtransform
{
  tf::Quaternion quat_temp = _latest.getRotation();
  tf::Vector3 vec_temp = _latest.getOrigin();

  if(!checkOrigin(vec_temp,_buffertf.getOrigin()) || !checkRotation(quat_temp,_buffertf.getRotation()))
  {
    _buffertf.setOrigin(vec_temp);
    _buffertf.setRotation(quat_temp);
  }
}

void filter_tf_class::filter_mv()
{
  tf::Vector3 vec_temp = _latest.getOrigin();
  tf::Vector3 vec_buff = _buffertf.getOrigin();

  tf::Vector3 _vec_new;
  _vec_new[0] = _alpha*vec_temp.x() + (1-_alpha)*vec_buff.x();
  _vec_new[1] = _alpha*vec_temp.y() + (1-_alpha)*vec_buff.y();
  _vec_new[2] = _alpha*vec_temp.z() + (1-_alpha)*vec_buff.z();
  _buffertf.setOrigin(_vec_new);

  if(_quat_change)
  {
      tf::Quaternion quat_temp = _latest.getRotation();
      tf::Quaternion quat_buff = _buffertf.getRotation();
      tf::Quaternion _quat_new;
      _quat_new.setX(_alpha*quat_temp.x() + (1-_alpha)*quat_buff.x());
      _quat_new.setY(_alpha*quat_temp.y() + (1-_alpha)*quat_buff.y());
      _quat_new.setZ(_alpha*quat_temp.z() + (1-_alpha)*quat_buff.z());
      _quat_new.setW(_alpha*quat_temp.w() + (1-_alpha)*quat_buff.w());
      _buffertf.setRotation(_quat_new);
  }
}

bool filter_tf_class::checkOrigin(tf::Vector3 input,tf::Vector3 input2)
{
  bool temp = true;
  if((abs(input.x() - input2.x()) > _vec_threshold) || (abs(input.y() - input2.y()) > _vec_threshold) || (abs(input.z() - input2.z()) > _vec_threshold))
  {
    temp = false;
  }
  return temp;
}

bool filter_tf_class::checkRotation(tf::Quaternion input,tf::Quaternion input2)
{
  bool temp = true;
  if((abs(input.w() - input2.w()) > _quat_threshold) || (abs(input.x() - input2.x()) > _quat_threshold) || (abs(input.y() - input2.y()) > _quat_threshold) || (abs(input.z() - input2.z()) > _quat_threshold))
  {
    temp = false;
  }
  return temp;
}

void filter_tf_class::run()
{
  _loop = new ros::Rate(_pub_rate);
  try
  {
    tf::StampedTransform temp;
    _listen.lookupTransform(_parent,_child,ros::Time(0),temp);
    _buffertf.setOrigin(temp.getOrigin());
    _buffertf.setRotation(temp.getRotation());
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }
  while(ros::ok() && _pnh->ok())
  {
    _listen.lookupTransform(_parent,_child,ros::Time(0),_latest);
    if(_alpha_run)
    {
      filter_mv();
    }
    else
    {
      filter();
    }
    _broadcast.sendTransform(tf::StampedTransform(_buffertf,ros::Time::now(),_new_parent,_new_child));
    _loop->sleep();
  }
}
