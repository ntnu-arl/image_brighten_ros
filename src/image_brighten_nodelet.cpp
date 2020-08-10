#include "ros/ros.h"
#include "image_brighten/image_brighten.h"

// Nodelet
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>


//NEW:
class imageBrightenNodelet : public nodelet::Nodelet {

  public:
    imageBrightenNodelet(){}
    ~imageBrightenNodelet(){}

  private:
    virtual void onInit() {

      nh_ = getNodeHandle();
      nh_private_ = getPrivateNodeHandle();

      imageBrighten *image_brighten_obj = new imageBrighten(nh_, nh_private_, "image_brighten_nodelet", 100);

      // RosInitialization here
    }

    ros::NodeHandle nh_, nh_private_; // Does not seem to need the ("~") in te nodelet case
};

//Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(imageBrightenNodelet, nodelet::Nodelet);