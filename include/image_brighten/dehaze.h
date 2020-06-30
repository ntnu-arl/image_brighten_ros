#ifndef HAZE_REMOVAL_H
#define HAZE_REMOVAL_H

#include <iostream>
#include "ros/ros.h"
#include "guided_filter.h"
#include "opencv2/opencv.hpp"
#include "opencv2/ximgproc.hpp"
#include <vector>


namespace dehaze
{
	typedef struct _pixel
	{
		int i;
		int j;
		uchar val;
		_pixel() : i(-1), j(-1), val((uchar)0) {}
		_pixel(int _i, int _j, uchar _val) : i(_i), j(_j), val((uchar)_val) {}
		bool operator<(const _pixel a) const
		{
			return a.val < val;
		}
	} Pixel;

	class CHazeRemoval
	{
	public:
		CHazeRemoval(int rows, int cols, int channels, int dark_ch_scale, int transmission_scale, float omega, float t0, int radius, int r, float eps, int filter_resize_factor);
		~CHazeRemoval();
		void Process(const cv::Mat *in_frame,cv::Mat &out_frame);
	private:
		void get_dark_channel();
		void get_dark_channel_old();
		void get_air_light();
		void get_transmission();
		void get_transmission_old();
		void guided_filter();
		void recover();

		int m_rows;
		int m_cols;
		int m_channels;

		int m_dehaze_radius;
		int m_guided_filter_radius;

		int m_dark_ch_resize_factor = 4;
		int m_transmission_resize_factor = 4;

		float m_omega;
		float m_t0;
		float m_eps;
		float m_filter_resize_factor;
		cv::Mat m_in_frame;

		cv::Vec3f m_vec3f_avg_light;

		std::vector<Pixel> m_light_pixel_vect;
		
		float* pix_mins = NULL;
		float* row_mins = NULL;

		GuidedFilterColor* m_guided_filter_ptr;

		cv::Mat m_dark_ch_resized_i;
		cv::Mat m_transmission_resized_i;
		cv::Mat m_cv_mat_transmission;
		cv::Mat m_cv_mat_guided_filter_output;
		cv::Mat m_cv_mat_recover_result;
		cv::Mat m_transmission_resized_result;
	};
} // namespace dehaze
#endif // !HAZE_REMOVAL_H