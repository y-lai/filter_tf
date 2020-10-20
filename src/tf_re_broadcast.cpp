#include "filter_tf/tf_re_broadcast.h"

tf_re_broadcast::tf_re_broadcast(ros::NodeHandle pnh):_robot_cam_height_override(true)
{
  _pnh = new ros::NodeHandle(pnh);
  if(!setup())
  {
    ROS_ERROR("Error when grabbing parameters. Please check roslaunch file.");
    return;
  }
  _loop = new ros::Rate(_pub_rate);
  _cam._marker = false;
  _robot._marker = false;

  _cam_marker_sub = _pnh->subscribe(_cam._marker_sub_topic.c_str(),1,&tf_re_broadcast::camMarkerCB,this);
  _robot_marker_sub =_pnh->subscribe(_robot._marker_sub_topic.c_str(),1,&tf_re_broadcast::robotMarkerCB,this);
  ROS_INFO("Spinning for callbacks!");
  while( (!_robot._marker || !_cam._marker) && ros::ok() && _pnh->ok())
  {
    ros::spinOnce();
    ros::Duration(2.5).sleep();
  }
  _cam_marker_sub.shutdown();
  _robot_marker_sub.shutdown();







  ROS_INFO("Obtained marker tfs. Inverting camera");
  ROS_INFO("Pre inverse TF: ");
  std::cout << _cam._broadcast_tf.getOrigin().getX() << ", " << _cam._broadcast_tf.getOrigin().getY() << ", " << _cam._broadcast_tf.getOrigin().getZ() << std::endl;
    _cam._broadcast_tf = _cam._broadcast_tf.inverse();
  ROS_INFO("Post inverse TF: ");
  std::cout << _cam._broadcast_tf.getOrigin().getX() << ", " << _cam._broadcast_tf.getOrigin().getY() << ", " << _cam._broadcast_tf.getOrigin().getZ() << std::endl;


  {
    ROS_INFO("Transforming robot tf by pi/2 around Z");
    tf::Quaternion quat_temp;
    quat_temp = tf::createQuaternionFromRPY(0,0,1.5708);
//    tf::Transform temp;
//    temp.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 1.5708));
    _robot._broadcast_tf.setRotation(_robot._broadcast_tf.getRotation()*quat_temp);
  }

  {
    std::string temp;
    std::getline(std::cin,temp);
  }

  ROS_INFO("Waiting for transform between base and %s",_robot._new_parent.c_str());
  {
    try
    {
      _listen.waitForTransform("/base",_robot._new_parent.c_str(),ros::Time(0),ros::Duration(5.0));
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    tf::StampedTransform temp;
    _listen.lookupTransform("/base",_robot._new_parent.c_str(),ros::Time(0),temp);
    _broadcasted.setOrigin(temp.getOrigin());
    _broadcasted.setRotation(temp.getRotation());
    ROS_INFO("TF between base and %s:",_robot._new_parent.c_str());
    std::cout << temp.getOrigin().getX() << ", " << temp.getOrigin().getY() << ", " << temp.getOrigin().getZ() << std::endl;
    std::cout << temp.getRotation().getW() << ", " << temp.getRotation().getX() << ", " << temp.getRotation().getY() << ", " << temp.getRotation().getZ() << std::endl;

    {
      tf::Transform temp = _robot._broadcast_tf;
      ROS_INFO("TF between %s and %s:",_robot._new_parent.c_str(),_robot._new_child.c_str());
      std::cout << temp.getOrigin().getX() << ", " << temp.getOrigin().getY() << ", " << temp.getOrigin().getZ() << std::endl;
      std::cout << temp.getRotation().getW() << ", " << temp.getRotation().getX() << ", " << temp.getRotation().getY() << ", " << temp.getRotation().getZ() << std::endl;
    }

    {
      tf::Transform temp = _cam._broadcast_tf;
      ROS_INFO("TF between %s and %s:",_cam._new_parent.c_str(),_cam._new_child.c_str());
      std::cout << temp.getOrigin().getX() << ", " << temp.getOrigin().getY() << ", " << temp.getOrigin().getZ() << std::endl;
      std::cout << temp.getRotation().getW() << ", " << temp.getRotation().getX() << ", " << temp.getRotation().getY() << ", " << temp.getRotation().getZ() << std::endl;
    }

    std::string temps;
    std::getline(std::cin,temps);


    _broadcasted = _broadcasted*_robot._broadcast_tf*_cam._broadcast_tf;
  }

  {
    tf::Vector3 temp = _broadcasted.getOrigin();
    tf::Quaternion tempq = _broadcasted.getRotation();
    ROS_INFO("Broadcasted tf: ");
    std::cout << temp.getX() << ", " << temp.getY() << ", " << temp.getZ() << std::endl;
    ROS_INFO("Broadcasted quaternion: ");
    std::cout << tempq.getW() << ", " << tempq.getX() << ", " << tempq.getY() << ", " << tempq.getZ() << std::endl;
  }

  {
    std::string temp;
    std::getline(std::cin,temp);
  }


  ROS_INFO("Running broadcasters!");
  while(ros::ok() && _pnh->ok())
  {
    _broadcast.sendTransform(tf::StampedTransform(_broadcasted,ros::Time::now(),"/base",_cam._new_child.c_str()));
  }

}

bool tf_re_broadcast::setup()
{
  bool temp = true;

  if(!_pnh->getParam("robot_new_parent_frame",_robot._new_parent))
  {
    ROS_INFO("Robot new parent frame argument not found!");
    temp = false;
  }
  if(!_pnh->getParam("cam_new_parent_frame",_cam._new_parent))
  {
    ROS_INFO("Cam new parent frame argument not found!");
    temp = false;
  }

  if(!_pnh->getParam("robot_new_child_frame",_robot._new_child))
  {
    ROS_INFO("Robot new child frame argument not found!");
    temp = false;
  }
  if(!_pnh->getParam("cam_new_child_frame",_cam._new_child))
  {
    ROS_INFO("Cam new child frame argument not found!");
    temp = false;
  }

  if(!_pnh->getParam("robot_inverse",_robot._inverse))
  {
    _robot._inverse = false;
  }
  if(!_pnh->getParam("cam_inverse",_cam._inverse))
  {
    _cam._inverse = false;
  }

  if(!_pnh->getParam("cam_marker_sub_topic",_cam._marker_sub_topic))
  {
    ROS_INFO("Cam marker sub topic argument not found!");
    temp = false;
  }
  if(!_pnh->getParam("robot_marker_sub_topic",_robot._marker_sub_topic))
  {
    ROS_INFO("Robot marker sub topic argument not found!");
    temp = false;
  }

  if(!_pnh->getParam("tf_publish_rate",_pub_rate))
  {
    ROS_INFO("Tf publish rate argument not found!");
    temp = false;
  }

  if(!_pnh->getParam("ar_marker_number",_ar_marker_number))
  {
    ROS_INFO("Ar marker number argument not found!");
    temp = false;
  }

  if(!_pnh->getParam("robot_cam_height",_robot_cam_height))
  {
    _robot_cam_height_override = false;
  }

  return temp;
}

void tf_re_broadcast::camMarkerCB(const ar_track_alvar_msgs::AlvarMarkersConstPtr &mark)
{
  for(std::vector<ar_track_alvar_msgs::AlvarMarker>::const_iterator it = mark->markers.begin();it != mark->markers.end(); it++)
  {
    if(it->id == (int) _ar_marker_number)
    {
      ROS_INFO("INSIDE CAMERA CB!!!!!");
      tf::Vector3 temp;
      temp.setX(it->pose.pose.position.x);
      temp.setY(it->pose.pose.position.y);
      temp.setZ(it->pose.pose.position.z);
      _cam._broadcast_tf.setOrigin(temp);

      tf::Quaternion tempq;
      tempq.setW(it->pose.pose.orientation.w);
      tempq.setX(it->pose.pose.orientation.x);
      tempq.setY(it->pose.pose.orientation.y);
      tempq.setZ(it->pose.pose.orientation.z);
      _cam._broadcast_tf.setRotation(tempq);

      _cam._marker = true;

      ROS_INFO("CAMERA CB");
      std::cout << temp.getX() << ", " << temp.getY() << ", " << temp.getZ() << std::endl;
      std::cout << tempq.getW() << ", " << tempq.getX() << ", " << tempq.getY() << ", " << tempq.getZ() << std::endl;

      return;
    }
  }
}

void tf_re_broadcast::robotMarkerCB(const ar_track_alvar_msgs::AlvarMarkersConstPtr &mark)
{
  for(std::vector<ar_track_alvar_msgs::AlvarMarker>::const_iterator it = mark->markers.begin();it != mark->markers.end(); it++)
  {
    if(it->id == (int) _ar_marker_number)
    {
      ROS_INFO("INSIDE ROBOT CB!!!!");
      tf::Vector3 temp;
      temp.setX(it->pose.pose.position.x);
      temp.setY(it->pose.pose.position.y);
      if(_robot_cam_height_override)
      {
        temp.setZ(_robot_cam_height);
      }
      else
      {
        temp.setZ(it->pose.pose.position.z);
      }
      _robot._broadcast_tf.setOrigin(temp);

      tf::Quaternion tempq;
      tempq.setW(it->pose.pose.orientation.w);
      tempq.setX(it->pose.pose.orientation.x);
      tempq.setY(it->pose.pose.orientation.y);
      tempq.setZ(it->pose.pose.orientation.z);
      _robot._broadcast_tf.setRotation(tempq);


      _robot._marker = true;

      ROS_INFO("ROBOT CB");
      std::cout << temp.getX() << ", " << temp.getY() << ", " << temp.getZ() << std::endl;
      std::cout << tempq.getW() << ", " << tempq.getX() << ", " << tempq.getY() << ", " << tempq.getZ() << std::endl;

      return;
    }
  }
}
