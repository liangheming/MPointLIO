#include "point_lio_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_lio");
    ros::NodeHandle nh("~");
    PointLIOROS point_lio_ros(nh);
    ros::spin();
    return 0;
}