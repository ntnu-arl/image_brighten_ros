#include "ros/ros.h"
#include "image_brighten/image_brighten.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_brighten");
    ros::NodeHandle nh("image_brighten");
    ros::NodeHandle nh_private("~");
    //ros::AsyncSpinner spinner(2);
    imageBrighten *image_brighten_obj = new imageBrighten(nh, nh_private, "image_brighten_node", 100);

    //spinner.start();
    //ros::waitForShutdown();
    ros::spin();
    return 0;
}