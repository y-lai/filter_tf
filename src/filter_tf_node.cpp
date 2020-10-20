#include <filter_tf/filter_tf_class.h>

int main(int argc,char** argv)
{
  ros::init(argc,argv,"filter_tf");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  filter_tf_class test(pnh,true);

//  if(argc > 1)
//  {
//    filter_tf_class test(pnh,true);
//  }
//  else
//  {
//    filter_tf_class test(pnh,false);
//  }

  return 1;
}
