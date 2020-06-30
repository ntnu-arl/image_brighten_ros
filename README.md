# Image Brighten ROS
ROS node implementing low light image enhancement.
The output image will have the same header as the input image.
The node will run at the parameterized rate, always processing the latest image. 
In the event computation speed cannot keep up with the desired rate, additional images will be dropped. 

##	Build and Run
To build and run an example, enter your catkin workspace and execute the following

    cd src
    git clone https://github.com/unr-arl/image_brighten_ros
    cd ..
    catkin build -DCMAKE_BUILD_TYPE=Release
    source devel/setup.bash
    roslaunch image_brighten image_brighten.launch topic_image_input:=/yourcam/imagetopicname
The node will create a new topic under *'[...]/bright'.*  We provide example ROSBAGs to allow the convenient testing of the method and its performance. 
    

##	Method Principle

Motivated by the method described in:

> Dong, X., Wang, G., Pang, Y., Li, W., Wen, J., Meng, W. and Lu, Y., 2011, July. Fast efficient algorithm for enhancement of low lighting video. In 2011 IEEE International Conference on Multimedia and Expo (pp. 1-6). IEEE.

And as discussed in an associated [Mathworks example](https://www.mathworks.com/help/images/low-light-image-enhancement.html), this node implements the following steps:

 - Step 1: Invert the input image.
 - Step 2: Apply haze removal to the inverted image. 
 - Step 3: Invert the enhanced image.

To speed-up, the method performs *rescaling* internally. As detailed in the paper and link above, this process is valid as the histogram of pixel-wise inversion of low-light images is very similar to the histogram of hazy images. This in turn implies that we can use haze remove methods to enhance low-light images. 


## Parameters
The following parameters allow to tune the algorithm to achieve the desired performance and computational load. 
 - **enable_dyn_reconf** : Enables or Disables Dynamic Reconfiguration.
 - **topic_image_input** : Image topic to which the node subscribes.
 - **topic_image_output** : Image topic to which the node publishes.
 - **enable_dehaze** : If disabled, node will publish resized image.
 - **max_process_freq** : Node will process images at a maximum of this frequency. Images received before the previous image is finished processing will be dropped. Otherwise the node will always take the latest image. 
 - **scale_factor** : Pre Proccessing scale factor, the image resolution will be dropped by this factor before any processing takes place.
 - **dark_ch_scale** : Scale factor applied before calculating the average dark channel value.
 - **transmission_scale** : Scale factor applied before calculating transmission image.
 - **dehaze_radius** : The neighborhood radius to be used to find the dark channel prior.
 - **dehaze_omega** : See Dark Channel Prior.
 - **dehaze_t0** : See Dark Channel Prior.
 - **dehaze_r** : Guided Filter Neighbor Radius.
 - **dehaze_eps** : See Guided Filter.
 - **dehaze_filter_resize_factor** : Resize factor for guided filter. Filter results are applied at the input resolution.

## Example ROSBAGs 
We provide two example ROSBAGs with dark image data to allow the testing of the method.
 - [ROSBAG #1](https://drive.google.com/file/d/1ZY-K0I0ZzVvjnCwb3Fr8HS9CLojSTcIv/view?usp=sharing)
 - [ROSBAG #2](https://drive.google.com/file/d/1AEw-GsMq2iI83mctI6su5A2qq9R__Lga/view?usp=sharing)

### Example Output
![](https://github.com/unr-arl/image_dehaze/blob/master/imgs/smaller_long_tunnel_low_res.gif)

## GPU
GPU Implementation Coming Soon!

## Credits
To develop this software package we relied or took inspiration from the implementations in:
 - [Original Dehaze (Dark Channel Prior + Guided Filter)](https://github.com/ZQPei/Haze_Removal_cpp)
 - [Fast Guided Filter Adapted From](http://kaiminghe.com/eccv10/) 

## References
> He, K., Sun, J. and Tang, X., 2012. Guided image filtering. IEEE transactions on pattern analysis and machine intelligence, 35(6), pp.1397-1409.

> He, K., Sun, J. and Tang, X., 2010. Single image haze removal using dark channel prior. IEEE transactions on pattern analysis and machine intelligence, 33(12), pp.2341-2353.

> Dong, X., Wang, G., Pang, Y., Li, W., Wen, J., Meng, W. and Lu, Y., 2011, July. Fast efficient algorithm for enhancement of low lighting video. In 2011 IEEE International Conference on Multimedia and Expo (pp. 1-6). IEEE.

## Contact
You can contact us for any question or remark:
* [Paolo De Petris](mailto:pdepetris@nevada.unr.edu)
* [Frank Mascarich](mailto:fmascarich@nevada.unr.edu)
* [Kostas Alexis](mailto:kalexis@unr.edu)
