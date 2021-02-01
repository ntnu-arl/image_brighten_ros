#include "ros/ros.h"
#include "image_brighten/dehaze.h"
#include <iostream>

#define GET_INDEX(x, y, cols)  (x * cols) + y
#define GET_MIN_VAL(r, g, b)   b > g ? ((g > r) ? r : g) : ((b > r) ? r : b)
#define GET_MIN(a,b)   a > b ? b : a
#define GET_MAX(a,b)   a < b ? b : a

#define OUTPUT_FUNCT_TIMING_STATS false

namespace dehaze
{

	static cv::Mat boxfilter(const cv::Mat &I, int r)
	{
		cv::Mat result;
		cv::blur(I, result, cv::Size(r, r));
		return result;
	}
	CHazeRemoval::CHazeRemoval(int rows, int cols, int channels, int dark_ch_scale, int transmission_scale, float omega, float t0, int radius, int r, float eps, int filter_resize_factor)
	{
		m_rows = rows;
		m_cols = cols;
		m_channels = channels;
		m_omega = omega;
		m_t0 = t0;
		m_dehaze_radius = radius;
		m_guided_filter_radius = r;
		m_eps = eps;

		m_filter_resize_factor = filter_resize_factor;
		m_dark_ch_resize_factor = dark_ch_scale;
		m_transmission_resize_factor = transmission_scale;

		m_dark_ch_resized_i = cv::Mat(m_rows / m_dark_ch_resize_factor, m_cols / m_dark_ch_resize_factor, CV_8UC3);
		m_transmission_resized_i = cv::Mat(m_rows / m_transmission_resize_factor, m_cols / m_transmission_resize_factor, CV_8UC3);
		m_transmission_resized_result = cv::Mat(m_rows / m_transmission_resize_factor, m_cols / m_transmission_resize_factor, CV_32FC1);
		m_cv_mat_transmission = cv::Mat(m_rows, m_cols, CV_32FC1);
		m_cv_mat_guided_filter_output = cv::Mat(m_rows, m_cols, CV_32FC1);
		m_cv_mat_recover_result = cv::Mat(m_rows, m_cols, CV_32FC3);

		m_light_pixel_vect.reserve(m_rows*m_cols);
		pix_mins = new float[m_rows*m_cols];
		row_mins = new float[m_rows*m_cols];
		m_guided_filter_ptr = new GuidedFilterColor(m_rows, m_cols, m_guided_filter_radius, m_eps, m_filter_resize_factor);
	}
	CHazeRemoval::~CHazeRemoval()
	{
		delete pix_mins;
		delete row_mins;
		delete m_guided_filter_ptr;
	}
	void CHazeRemoval::Process(const cv::Mat *in_frame,cv::Mat &out_frame)
	{
		m_in_frame = in_frame->clone();
			#if OUTPUT_FUNCT_TIMING_STATS
				clock_t start = clock();
			#endif
		get_dark_channel();
			#if OUTPUT_FUNCT_TIMING_STATS
				std::cout << "Get Dark Channel Time = " << float(clock() - start) / CLOCKS_PER_SEC << std::endl;
				start = clock();
			#endif
		get_air_light();
			#if OUTPUT_FUNCT_TIMING_STATS
				std::cout << "Get A Light Time = " << float(clock() - start) / CLOCKS_PER_SEC << std::endl;
				start = clock();
			#endif
		get_transmission();
			#if OUTPUT_FUNCT_TIMING_STATS
				std::cout << "Get Transmission Time = " << float(clock() - start) / CLOCKS_PER_SEC << std::endl;
				start = clock();
			#endif
		guided_filter();
			#if OUTPUT_FUNCT_TIMING_STATS
				std::cout << "Run Guided Filter Time = " << float(clock() - start) / CLOCKS_PER_SEC << std::endl;
				start = clock();
			#endif
		recover();
			#if OUTPUT_FUNCT_TIMING_STATS
				std::cout << "Recover Filter Results Time = " << float(clock() - start) / CLOCKS_PER_SEC << std::endl;
			#endif

		m_cv_mat_recover_result.convertTo(out_frame, CV_8UC3);
	}
	void CHazeRemoval::get_dark_channel()
	{
		m_light_pixel_vect.clear();
		cv::resize(m_in_frame, m_dark_ch_resized_i, m_dark_ch_resized_i.size(), 0, 0, INTER_NN);
		/*
			1. Iterate over each pixel and save the smallest rgb vals for each pixel in pix_mins
			2. Iterate over each pixel, Iterate over that pixel's row neighbors, save the smallest value in row_mins
			3. Iterate over each pixel, Iterate over that pixel's column neighbors, save the smallest value in the light_pixels_vector
		*/
		unsigned long px_idx = 0;
		// for each pixel, calculate the min rgb
		for (int i = 0; i < m_dark_ch_resized_i.rows; ++i)
		{
			for (int j = 0; j < m_dark_ch_resized_i.cols; ++j)
			{
				// get the vector of pixel values
				cv::Vec3b tmp = m_dark_ch_resized_i.ptr<cv::Vec3b>(i)[j];
				// save the smallest value at the corresponding array location
				pix_mins[px_idx] = GET_MIN_VAL(float(tmp[0]), float(tmp[1]), float(tmp[2]));
				px_idx++;
			}
		}
		// create an array to store the min row values (min of each row of neighbors)
		px_idx = 0;
		// for each pixel, calculate the min of the row neighbors
		for (int i = 0; i < m_dark_ch_resized_i.rows; ++i)
		{
			// calculate the neighbor bounds on the row neighbors
				int rmin = cv::max(0, i - m_dehaze_radius);
				int rmax = cv::min(i + m_dehaze_radius, m_dark_ch_resized_i.rows - 1);
			// loop through the columns
			for (int j = 0; j < m_dark_ch_resized_i.cols; ++j)
			{
				float min_val = 255.0f;
				// loop over the bytes in the row
				for(int k = rmin; k <= rmax; ++k)
				{
					min_val = cv::min((float)pix_mins[GET_INDEX(k, j, m_dark_ch_resized_i.cols)], min_val);
				}
				row_mins[px_idx] = min_val;
				px_idx++;
			}
		}
		// for each pixel, calculate the min of the column neighbors
		for (int i = 0; i < m_dark_ch_resized_i.rows; ++i)
		{
			for (int j = 0; j < m_dark_ch_resized_i.cols; ++j)
			{
				int cmin = cv::max(0, j - m_dehaze_radius);
				int cmax = cv::min(j + m_dehaze_radius, m_dark_ch_resized_i.cols - 1);
				float min_val = 255.0f;
				// iterate over the column
				for(int k = cmin; k <= cmax; ++k)
				{
					min_val = cv::min((float)pix_mins[GET_INDEX(i, k, m_dark_ch_resized_i.cols)], min_val);
				}
				m_light_pixel_vect.push_back(Pixel(i, j, min_val));
			}
		}
	}

	// Gets the average of the top few pixels (0.1%) of m_light_pixel_vect
	void CHazeRemoval::get_air_light()
	{
		int num_pixels = int(m_dark_ch_resized_i.rows * m_dark_ch_resized_i.cols * 0.001);
		std::vector<Pixel> best_light_pixel_vect(num_pixels);
		std::partial_sort_copy(m_light_pixel_vect.begin(), m_light_pixel_vect.end(), best_light_pixel_vect.begin(), best_light_pixel_vect.end());
		float A_sum[3] = {0.0f,0.0f,0.0f};
		std::vector<Pixel>::iterator it = best_light_pixel_vect.begin();
		for (int cnt = 0; cnt < num_pixels; cnt++)
		{
			cv::Vec3b tmp = m_dark_ch_resized_i.ptr<cv::Vec3b>(it->i)[it->j];
			A_sum[0] += tmp[0];
			A_sum[1] += tmp[1];
			A_sum[2] += tmp[2];
			it++;
		}
		for (int i = 0; i < 3; i++)
		{
			m_vec3f_avg_light[i] = A_sum[i] / (float)num_pixels;
		}
	}
	void CHazeRemoval::get_transmission()
	{
		cv::resize(m_in_frame, m_transmission_resized_i, m_transmission_resized_i.size(), 0, 0, INTER_NN);
		// pre calculate the minimum pixel values for each pixel, so they're calculated n times, not n*m times
		unsigned long int pix_idx = 0;
		for (int i = 0; i < m_transmission_resized_i.rows; ++i)
		{
			for (int j = 0; j < m_transmission_resized_i.cols; ++j)
			{
				cv::Vec3b tmp = m_transmission_resized_i.ptr<cv::Vec3b>(i)[j];
				float b = (float)tmp[0] / m_vec3f_avg_light[0];
				float g = (float)tmp[1] / m_vec3f_avg_light[1];
				float r = (float)tmp[2] / m_vec3f_avg_light[2];
				pix_mins[pix_idx] = GET_MIN_VAL(b,g,r);
				pix_idx++;
			}
		}
		// create an array to store the min row values (min of each row of neighbors)
		pix_idx = 0;
		// for each pixel, calculate the min of the row neighbors
		for (int i = 0; i < m_transmission_resized_i.rows; ++i)
		{
			int min_i = cv::max(0, i-m_dehaze_radius);
			int max_i = cv::min(m_transmission_resized_i.rows -1, i+m_dehaze_radius);
			for (int j = 0; j < m_transmission_resized_i.cols; ++j)
			{
				int px_idx = GET_INDEX(i, j, m_transmission_resized_i.cols);
				float row_min = 255.0;
				// loop over the bytes in the row
				for(int k = min_i; k <= max_i; ++k)
				{
					row_min = GET_MIN(pix_mins[GET_INDEX(k, j, m_transmission_resized_i.cols)], row_min);
				}
				row_mins[px_idx] = row_min;
				pix_idx++;
			}
		}
		// for each pixel, calculate the min of the column neighbors
		for (int i = 0; i < m_transmission_resized_i.rows; ++i)
		{
			for (int j = 0; j < m_transmission_resized_i.cols; ++j)
			{
				int min_j = cv::max(0, j-m_dehaze_radius);
				int max_j = cv::min(m_transmission_resized_i.cols-1, j+m_dehaze_radius);
				float min_val = 255.0;
				// iterate over the column
				for(int k = min_j; k <= max_j; ++k)
				{
					int idx = GET_INDEX(i, k, m_transmission_resized_i.cols);
					min_val = GET_MIN(row_mins[idx], min_val);
				}
				m_transmission_resized_result.ptr<float>(i)[j] = 1.0f - m_omega * min_val;
			}
		}
		cv::resize(m_transmission_resized_result, m_cv_mat_transmission, m_in_frame.size(), 0,0, INTER_LINEAR);
	}
	void CHazeRemoval::guided_filter()
	{
		m_guided_filter_ptr->filter(m_in_frame, m_cv_mat_transmission, m_cv_mat_guided_filter_output);
	}
	void CHazeRemoval::recover()
	{
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				cv::Vec3f result = cv::Vec3f(m_in_frame.ptr<cv::Vec3b>(i)[j]) - m_vec3f_avg_light;
				result /= cv::max(m_t0, m_cv_mat_guided_filter_output.ptr<float>(i)[j]);
				result += m_vec3f_avg_light;
				m_cv_mat_recover_result.ptr<cv::Vec3f>(i)[j] = result;
			}
		}
		m_cv_mat_recover_result = cv::min(m_cv_mat_recover_result, 255.0);
		m_cv_mat_recover_result = cv::max(m_cv_mat_recover_result, 0.0);
	}
} // namespace dehaze
