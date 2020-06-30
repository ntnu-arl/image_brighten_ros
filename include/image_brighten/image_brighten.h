#pragma once

#include "ros/ros.h"
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_brighten/ImageBrightenConfig.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include "image_brighten/dehaze.h"
#include "image_brighten/guided_filter.h"
#include <mutex>

class imageBrighten
{

public:
	imageBrighten(ros::NodeHandle &n, const std::string &s, int bufSize);
	~imageBrighten();
	void callback_image_input(const sensor_msgs::ImageConstPtr &msg);
	void callback_dyn_reconf(image_brighten::ImageBrightenConfig &config, uint32_t level);
	// dynamic reconfigure
	dynamic_reconfigure::Server<image_brighten::ImageBrightenConfig> _dr_srv;
	dynamic_reconfigure::Server<image_brighten::ImageBrightenConfig>::CallbackType _dyn_rec_cb;
	void process_next_image(const ros::TimerEvent& event);

private:
	image_transport::ImageTransport m_image_transport;
	image_transport::Publisher m_pub_brighten_image;
	image_transport::Subscriber m_sub_image;
	unsigned long int m_frame_counter = 0;
	typedef boost::mutex::scoped_lock lock_t;
	boost::mutex m_image_buffer_mutex;
	boost::circular_buffer<sensor_msgs::ImageConstPtr> m_image_buffer;

	boost::shared_ptr<dehaze::CHazeRemoval> haze_removal_obj_ptr;

	// parameters
	std::string m_topic_image_input;
	std::string m_topic_image_output;

	cv::Size m_image_proc_size;

	bool m_enable_dyn_reconf;
	int m_scale_factor;
	bool m_enable_brighten;
	bool m_run_max_fps;
	double m_proc_freq;


	ros::Timer process_timer;

	boost::mutex dehaze_obj_mutex;

	// Dehaze Params
	int m_dark_ch_scale;
	int m_transmission_scale;
	int m_dehaze_radius;
	double m_dehaze_omega;
	double m_dehaze_t0;
	int m_guided_filter_radius;
	double m_guided_filter_eps;
	int m_guided_filter_resize_factor;


};
