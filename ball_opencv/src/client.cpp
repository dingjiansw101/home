#include "ros/ros.h"
#include "ball_opencv/ball_direction.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ball_direction_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ball_opencv::ball_direction>("ball_direction");
  ball_opencv::ball_direction srv;
  //srv.request.a = atoll(argv[1]);
  //srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("direction: %ld", (long int)srv.response.direction);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
