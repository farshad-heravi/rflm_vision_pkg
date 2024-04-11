#include <iostream>
#include <Camera.hpp>
#include "ros/ros.h"
// #include "geometry_msgs/msg/pose_stamped.h"


int main(int argc, char* argv[])
{
    int fps = 60;
    int lens_position = 150;
    int marker_color = 1, object_color = 2;     // 0 for red; 1 for yellow; 2 for green
    bool realtime_visualization = false;        // whether to show the image while working

    Camera cam = Camera(fps, lens_position, marker_color, object_color, realtime_visualization);
    cam.calibrate();    // it is a blocking method, will continue after the calibration is done

    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1000);

    // ros::Publisher link_pos_publisher = nh.advertise<geometry_msgs::PoseStamped>("cameraSensor", 10);
    // ros::Publisher object_info_publisher = nh.advertise<geometry_msgs::PoseStamped>("objectDetector", 10);





    return 0;
}