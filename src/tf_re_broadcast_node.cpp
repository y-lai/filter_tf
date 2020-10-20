#include <filter_tf/tf_re_broadcast.h>


int main(int argc, char** argv)
{
  ros::init(argc,argv,"tf_re_broadcast_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tf_re_broadcast test(pnh);

  return 1;
}
