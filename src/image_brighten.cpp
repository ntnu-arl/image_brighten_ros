#include "image_brighten/image_brighten.h"


#define OUTPUT_TIMING_STATS false

imageBrighten::imageBrighten(ros::NodeHandle &n, ros::NodeHandle &n_private, const std::string &s, int bufSize): m_image_buffer(1), m_image_transport(n)
{
	// parameters
	n.getParam("enable_dyn_reconf", m_enable_dyn_reconf);

	if (m_enable_dyn_reconf)
	{
		_dyn_rec_cb = boost::bind(&imageBrighten::callback_dyn_reconf, this, _1, _2);
		_dr_srv.setCallback(_dyn_rec_cb);
	}

	// ros parameters
	m_topic_image_input=  std::string("/cam0/cam0");
	n_private.getParam("topic_image_input", m_topic_image_input);
	m_topic_image_output =  m_topic_image_input + "/bright";
	n_private.getParam("topic_image_output", m_topic_image_output);
	m_scale_factor =  2;
	n.getParam("scale_factor", m_scale_factor);
	m_enable_brighten =  true;
	n.getParam("enable_brighten", m_enable_brighten);
	m_proc_freq =  1.0;
	n.getParam("max_process_freq", m_proc_freq);

	// dehaze Parameters
	m_dark_ch_scale = 4;
	n.getParam("dark_ch_scale", m_dark_ch_scale);
	m_transmission_scale = 4;
	n.getParam("transmission_scale", m_transmission_scale);
	m_dehaze_radius =  5;
	n.getParam("dehaze_radius", m_dehaze_radius);
	m_dehaze_omega =  0.85;
	n.getParam("dehaze_omega", m_dehaze_omega);
	m_dehaze_t0 =  0.1;
	n.getParam("dehaze_t0", m_dehaze_t0);
	
	// guided filter Parameters
	m_guided_filter_radius =  25;
	n.getParam("guided_filter_radius", m_guided_filter_radius);
	m_guided_filter_eps =  0.05;
	n.getParam("guided_filter_eps", m_guided_filter_eps);
	m_guided_filter_resize_factor =  4;
	n.getParam("guided_filter_resize_factor", m_guided_filter_resize_factor);

	m_sub_image = m_image_transport.subscribe(m_topic_image_input, 10, &imageBrighten::callback_image_input, this);
	m_pub_brighten_image = m_image_transport.advertise(m_topic_image_output, 10);
	process_timer = n.createTimer(m_proc_freq, &imageBrighten::process_next_image, this);
}

imageBrighten::~imageBrighten()
{

}
void imageBrighten::callback_image_input(const sensor_msgs::ImageConstPtr &msg)
{
	
	lock_t scope_lock(m_image_buffer_mutex);
	m_image_buffer.push_back(msg);
	m_frame_counter++;
}

void imageBrighten::process_next_image(const ros::TimerEvent& event)
{
	try
	{
		lock_t scope_lock(m_image_buffer_mutex);
		if(m_image_buffer.size() <= 0)
		{
			return;
		}
		sensor_msgs::ImageConstPtr msg = m_image_buffer[0];
		m_image_buffer.pop_front();
		scope_lock.unlock();

		cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

		cv::Mat image_in;

		// convert to color if not color already
		if (frame.channels() > 1)image_in = frame;
		else cv::cvtColor(image_in, frame, cv::COLOR_GRAY2BGR);

		// calculate the size of the processing image
		m_image_proc_size = cv::Size(image_in.cols / m_scale_factor, image_in.rows / m_scale_factor);

		// resize if scale factor isn't 1
		if(m_scale_factor != 1.0)
		{
			cv::resize(image_in, image_in, m_image_proc_size, 0, 0, CV_INTER_LINEAR);
		}

		cv::Mat image_out(image_in.rows, image_in.cols, CV_8UC3);
		if (m_enable_brighten)
		{
				ROS_WARN_ONCE("Brighten Enabled");
				#if OUTPUT_TIMING_STATS
					clock_t alg_start = clock();
				#endif
				// invert the image
				cv::Mat inverted_image =  cv::Scalar::all(255) - image_in;

    			boost::mutex::scoped_lock lock(dehaze_obj_mutex);
				// initialize dehazing if the pointer isn't already initialized
				if(!haze_removal_obj_ptr)
				{
					// std::cout << "Rebuilding" << std::endl;
					haze_removal_obj_ptr = boost::make_shared<dehaze::CHazeRemoval>(m_image_proc_size.height, m_image_proc_size.width, inverted_image.channels(),
																m_dark_ch_scale, m_transmission_scale,
																m_dehaze_omega, m_dehaze_t0, m_dehaze_radius, 
																m_guided_filter_radius, m_guided_filter_eps, 
																m_guided_filter_resize_factor);
				}
				// run dehazing
				haze_removal_obj_ptr->Process(&inverted_image, image_out);
				lock.unlock();
				// re-invert the image
				image_out = cv::Scalar::all(255) - image_out;
				#if OUTPUT_TIMING_STATS
					ROS_INFO("Brighten Process Time consumed : %f sec", (float)(clock() - alg_start) / CLOCKS_PER_SEC );
				#endif
		}
		else
		{
			ROS_WARN_ONCE("Brighten Disabled");
			image_out = image_in;
		}
		if(m_pub_brighten_image.getNumSubscribers() > 0)
		{
			sensor_msgs::Image::Ptr output_msg = cv_bridge::CvImage(msg->header, "bgr8", image_out).toImageMsg();
			m_pub_brighten_image.publish(output_msg);
		}
	}
	catch (cv::Exception &e)
	{
		ROS_WARN("Error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
	}
}

void imageBrighten::callback_dyn_reconf(image_brighten::ImageBrightenConfig &config, uint32_t level)
{
	ROS_INFO("Dynamic Reconfigure Triggered");
	m_enable_brighten = config.enable_brighten;
	m_dehaze_radius = config.dehaze_radius;
	m_dehaze_omega = config.dehaze_omega;
	m_dehaze_t0 = config.dehaze_t0;
	m_guided_filter_radius = config.guided_filter_radius;
	m_guided_filter_eps = config.guided_filter_eps;
	m_dark_ch_scale = config.dark_ch_scale;
	m_transmission_scale = config.transmission_scale;
	m_scale_factor = config.scale_factor;
	m_guided_filter_resize_factor = config.guided_filter_resize_factor;
	boost::mutex::scoped_lock lock(dehaze_obj_mutex);
	haze_removal_obj_ptr.reset();
	
}
