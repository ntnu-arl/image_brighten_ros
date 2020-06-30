#ifndef GUIDED_FILTER_H
#define GUIDED_FILTER_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


class GuidedFilterColor
{
	public:
		GuidedFilterColor(int rows, int cols, int r, double eps, int resize_factor);
		void filter(const cv::Mat &I, const cv::Mat &p, cv::Mat &result);
	private:
		std::vector<cv::Mat> Ichannels;
		int m_radius;
		int m_sub_radius;
		cv::Size m_resized_size, m_original_image_size;
		double m_eps;
		int m_original_I_depth;
		cv::Mat mean_I_r, mean_I_g, mean_I_b;
		cv::Mat invrr, invrg, invrb, invgg, invgb, invbb;
		cv::Mat resized_I;
		cv::Mat reformated_I;
		cv::Mat var_I_rr;
		cv::Mat var_I_rg;
		cv::Mat var_I_rb;
		cv::Mat var_I_gg;
		cv::Mat var_I_gb;
		cv::Mat var_I_bb;
		cv::Mat covDet;
		cv::Mat resized_p;
		cv::Mat mean_p;
		cv::Mat mean_Ip_r;
		cv::Mat mean_Ip_g;
		cv::Mat mean_Ip_b;
		cv::Mat cov_Ip_r;
		cv::Mat cov_Ip_g;
		cv::Mat cov_Ip_b;
		cv::Mat a_r;
		cv::Mat a_g;
		cv::Mat a_b;
		cv::Mat b;
		cv::Mat temp;
};

#endif